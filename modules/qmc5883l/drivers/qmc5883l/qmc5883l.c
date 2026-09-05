#define DT_DRV_COMPAT qst_qmc5883l

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "qmc5883l.h"

LOG_MODULE_REGISTER(qmc5883l, LOG_LEVEL_INF);

// The registers of qmc5883l
#define QMC5883L_REG_OUT_X_LSB  0x00    /* data start: burst-read 6 bytes,
                                           X/Y/Z, 16-bit signed, LSB first */
#define QMC5883L_REG_STATUS     0x06    /* DOR | OVL | DRDY */
#define QMC5883L_REG_TEMP_LSB   0x07
#define QMC5883L_REG_TEMP_MSB   0x08
#define QMC5883L_REG_CONFIG_1   0x09    /* OSR | RNG | ODR | MODE */
#define QMC5883L_REG_CONFIG_2   0x0A    /* SOFT_RST | ROL_PNT | INT_ENB */
#define QMC5883L_REG_SETRST     0x0B    /* SET/RESET period, must be 0x01 */
#define QMC5883L_REG_ID         0x0D    /* chip id register */

// The values of qmc5883l
#define QMC5883L_NORMAL_ID      0xFF    /* expected chip id */
#define QMC5883L_STATUS_DRDY    BIT(0)  /* 1 = new sample ready */
/* CONFIG_1 = 0x19: OSR=512, RNG=+/-8Gs, ODR=100Hz, MODE=continuous */
#define QMC5883L_CONFIG_1_VAL   0x19
/* Max wait for DRDY: at 100Hz ODR a new sample arrives every 10ms */
#define QMC5883L_DRDY_TIMEOUT_MS 50

// Configure
struct qmc5883l_i2c_config {
    struct i2c_dt_spec i2c;
};

// Data
struct qmc5883l_i2c_data {
    struct k_mutex lock;
    uint8_t id;
};

static int qmc5883l_write_reg(const struct qmc5883l_i2c_config *cfg,
                             uint8_t reg, uint8_t val)
{
    int ret = i2c_reg_write_byte_dt(&cfg->i2c, reg, val);

    if (ret) {
        LOG_ERR("write reg 0x%02x = 0x%02x failed: %d", reg, val, ret);
    }
    return ret;
}

static int qmc5883l_read_data(const struct qmc5883l_i2c_config *cfg,
                            uint8_t reg, uint8_t *read, size_t len)
{
    int ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, read, len);
    if (ret) {
        LOG_ERR("read reg 0x%02x len %zu failed: %d", reg, len, ret);
    }
    return ret;
}

static int qmc5883l_read_id(const struct device *dev, uint8_t *id)
{
    const struct qmc5883l_i2c_config *cfg = dev->config;
    struct qmc5883l_i2c_data *data = dev->data;
    uint8_t rx;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = qmc5883l_read_data(cfg, QMC5883L_REG_ID, &rx, 1);
    k_mutex_unlock(&data->lock);
    if (ret == 0) {
        *id = rx;
    }
    return ret;
}

/* Read one magnetic sample. Waits for DRDY (bit0 of STATUS 0x06) so
 * we never return the same stale data twice, then burst-reads
 * X/Y/Z from 0x00 in one transaction. LSB first, unlike MPU6050. */
static int qmc5883l_read_mag(const struct device *dev, int16_t mag[3])
{
    const struct qmc5883l_i2c_config *cfg = dev->config;
    struct qmc5883l_i2c_data *data = dev->data;
    uint8_t buf[6], status;
    int64_t deadline = k_uptime_get() + QMC5883L_DRDY_TIMEOUT_MS;
    int ret;

    if (mag == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    /* Wait until a fresh sample is available */
    do {
        ret = qmc5883l_read_data(cfg, QMC5883L_REG_STATUS, &status, 1);
        if (ret) {
            goto out;
        }
        if (status & QMC5883L_STATUS_DRDY) {
            break;
        }
        if (k_uptime_get() >= deadline) {
            LOG_ERR("DRDY timeout, status 0x%02x", status);
            ret = -ETIMEDOUT;
            goto out;
        }
        k_msleep(1);
    } while (1);

    ret = qmc5883l_read_data(cfg, QMC5883L_REG_OUT_X_LSB, buf, sizeof(buf));
    if (ret) {
        goto out;
    }
    mag[0] = (int16_t)((buf[1] << 8) | buf[0]);
    mag[1] = (int16_t)((buf[3] << 8) | buf[2]);
    mag[2] = (int16_t)((buf[5] << 8) | buf[4]);
    ret = 0;

out:
    k_mutex_unlock(&data->lock);
    return ret;
}

// Initialize qmc5883l
static int qmc5883l_init(const struct device *dev)
{
    const struct qmc5883l_i2c_config *cfg = dev->config;
    struct qmc5883l_i2c_data *data = dev->data;
    uint8_t id;
    int ret;

    // Check i2c
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    // Init mutex
    k_mutex_init(&data->lock);

    // Software reset (bit7 of CONFIG_2), all registers back to default
    ret = qmc5883l_write_reg(cfg, QMC5883L_REG_CONFIG_2, 0x80);
    if (ret) {
        return ret;
    }
    k_msleep(100);

    /* SET/RESET period = 0x01: degauss the sensor element every sample
     * period, clears offset built up by strong external fields.
     * Skipping this is the classic QMC5883L mistake - data reads
     * garbage without it. */
    ret = qmc5883l_write_reg(cfg, QMC5883L_REG_SETRST, 0x01);
    if (ret) {
        return ret;
    }

    // Continuous mode, 100Hz, +/-8Gs, OSR=512
    ret = qmc5883l_write_reg(cfg, QMC5883L_REG_CONFIG_1,
                             QMC5883L_CONFIG_1_VAL);
    if (ret) {
        return ret;
    }

    // Read id: warn only on mismatch, clones often return other values
    ret = qmc5883l_read_id(dev, &id);
    if (ret) {
        return ret;
    }
    if (id != QMC5883L_NORMAL_ID) {
        LOG_WRN("unexpected ID 0x%02x (expected 0x%02x), continuing",
                id, QMC5883L_NORMAL_ID);
    } else {
        LOG_INF("QMC5883L detected, ID = 0x%02x", id);
    }
    data->id = id;

    LOG_INF("QMC5883L init done");
    return 0;
}

// Define the public API functions for the driver
static const struct qmc5883l_api qmc5883l_api_funcs = {
    .read_mag = qmc5883l_read_mag,
};

// Instance boilerplate

#define QMC5883L_I2C_INIT(inst)                                              \
    static const struct qmc5883l_i2c_config qmc5883l_i2c_cfg_##inst = {     \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
    };                                                                     \
    static struct qmc5883l_i2c_data qmc5883l_i2c_data_##inst;              \
                                                                           \
    DEVICE_DT_INST_DEFINE(inst,                                            \
                        qmc5883l_init,                                     \
                        NULL,                                              \
                        &qmc5883l_i2c_data_##inst,                         \
                        &qmc5883l_i2c_cfg_##inst,                          \
                        POST_KERNEL,                                       \
                        80,                                                \
                        &qmc5883l_api_funcs);                              \

DT_INST_FOREACH_STATUS_OKAY(QMC5883L_I2C_INIT)
