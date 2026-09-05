#define DT_DRV_COMPAT sensirion_sht30

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "sht30.h"

LOG_MODULE_REGISTER(sht30, LOG_LEVEL_INF);

/* The SHT30 has no register map: every operation is a 16-bit command
 * sent MSB first, then the chip answers with data on a plain read. */
#define SHT30_CMD_SINGLE_HIGH   0x2400  /* single shot, high repeatability,
                                           clock stretching disabled */
#define SHT30_CMD_READ_STATUS   0xF32D  /* read status register */
#define SHT30_CMD_CLEAR_STATUS  0x3041  /* clear status register */
#define SHT30_CMD_SOFT_RESET    0x30A2

/* Every command word and both data words are protected by CRC-8,
 * polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0xFF, no final XOR */
#define SHT30_CRC_POLY  0x31
#define SHT30_CRC_INIT  0xFF

/* High repeatability single shot needs at most 15ms (datasheet 3.6.2),
 * a bit of margin costs nothing */
#define SHT30_MEAS_TIME_MS      20
/* Datasheet 4.4: the chip needs ~1.5ms after soft reset */
#define SHT30_RESET_TIME_MS     2

// Configure
struct sht30_i2c_config {
    struct i2c_dt_spec i2c;
};

// Data
struct sht30_i2c_data {
    struct k_mutex lock;
    uint16_t status;
};

/* Send a 16-bit command, MSB first */
static int sht30_write_cmd(const struct sht30_i2c_config *cfg, uint16_t cmd)
{
    uint8_t buf[2] = { cmd >> 8, cmd & 0xFF };
    int ret = i2c_write_dt(&cfg->i2c, buf, sizeof(buf));

    if (ret) {
        LOG_ERR("write cmd 0x%04x failed: %d", cmd, ret);
    }
    return ret;
}

/* CRC-8 over len bytes, matches the datasheet example vectors */
static uint8_t sht30_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = SHT30_CRC_INIT;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ SHT30_CRC_POLY)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Trigger one single-shot measurement and read the 6 result bytes:
 * T_msb T_lsb CRC RH_msb RH_lsb CRC */
static int sht30_measure_raw(const struct sht30_i2c_config *cfg,
                             uint8_t buf[6])
{
    int ret = sht30_write_cmd(cfg, SHT30_CMD_SINGLE_HIGH);

    if (ret) {
        return ret;
    }

    /* The conversion runs while we sleep: with clock stretching
     * disabled the chip NACKs reads until the result is ready */
    k_msleep(SHT30_MEAS_TIME_MS);

    ret = i2c_read_dt(&cfg->i2c, buf, 6);
    if (ret) {
        LOG_ERR("read measurement failed: %d", ret);
        return ret;
    }

    /* A wrong CRC means a corrupted frame: fail instead of silently
     * converting garbage into plausible-looking values */
    if (sht30_crc8(buf, 2) != buf[2]) {
        LOG_ERR("temperature CRC mismatch");
        return -EIO;
    }
    if (sht30_crc8(buf + 3, 2) != buf[5]) {
        LOG_ERR("humidity CRC mismatch");
        return -EIO;
    }
    return 0;
}

static int sht30_read_th(const struct device *dev, float *temp_c,
                         float *hum_pct)
{
    const struct sht30_i2c_config *cfg = dev->config;
    struct sht30_i2c_data *data = dev->data;
    uint8_t buf[6];
    uint16_t st, srh;
    int ret;

    if (temp_c == NULL || hum_pct == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    ret = sht30_measure_raw(cfg, buf);
    if (ret == 0) {
        st = ((uint16_t)buf[0] << 8) | buf[1];
        srh = ((uint16_t)buf[3] << 8) | buf[4];
        /* Datasheet 4.13: T[degC] = -45 + 175 * S_T / 65535,
         * RH[%] = 100 * S_RH / 65535 */
        *temp_c = -45.0f + 175.0f * (float)st / 65535.0f;
        *hum_pct = 100.0f * (float)srh / 65535.0f;
        /* %RH is only defined 0..100, clamp noise near saturation */
        if (*hum_pct > 100.0f) {
            *hum_pct = 100.0f;
        }
    }

    k_mutex_unlock(&data->lock);
    return ret;
}

static int sht30_read_status(const struct device *dev, uint16_t *status)
{
    const struct sht30_i2c_config *cfg = dev->config;
    struct sht30_i2c_data *data = dev->data;
    uint8_t buf[3];
    int ret;

    if (status == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = sht30_write_cmd(cfg, SHT30_CMD_READ_STATUS);
    if (ret == 0) {
        ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    }
    k_mutex_unlock(&data->lock);
    if (ret == 0) {
        if (sht30_crc8(buf, 2) != buf[2]) {
            LOG_ERR("status CRC mismatch");
            return -EIO;
        }
        *status = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

static int sht30_reset(const struct device *dev)
{
    const struct sht30_i2c_config *cfg = dev->config;
    struct sht30_i2c_data *data = dev->data;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = sht30_write_cmd(cfg, SHT30_CMD_SOFT_RESET);
    k_mutex_unlock(&data->lock);
    if (ret == 0) {
        k_msleep(SHT30_RESET_TIME_MS);
    }
    return ret;
}

// Initialize sht30
static int sht30_init(const struct device *dev)
{
    const struct sht30_i2c_config *cfg = dev->config;
    struct sht30_i2c_data *data = dev->data;
    uint16_t status;
    int ret;

    // Check i2c
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    // Init mutex
    k_mutex_init(&data->lock);

    // Soft reset: sensor back to default state, ~1.5ms
    ret = sht30_reset(dev);
    if (ret) {
        return ret;
    }

    /* Read the status register once: proves the chip actually answers.
     * Bit1 (reset detected) is always set right after a soft reset,
     * that one is expected and not an error. */
    ret = sht30_read_status(dev, &status);
    if (ret) {
        return ret;
    }
    data->status = status;
    if (status & BIT(15)) {
        LOG_WRN("alert pending, status 0x%04x", status);
    }

    LOG_INF("SHT30 init done, status 0x%04x", status);
    return 0;
}

// Define the public API functions for the driver
static const struct sht30_api sht30_api_funcs = {
    .read_th = sht30_read_th,
    .read_status = sht30_read_status,
    .reset = sht30_reset,
};

// Instance boilerplate

#define SHT30_I2C_INIT(inst)                                            \
    static const struct sht30_i2c_config sht30_i2c_cfg_##inst = {       \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                              \
    };                                                                  \
    static struct sht30_i2c_data sht30_i2c_data_##inst;                 \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                        sht30_init,                                     \
                        NULL,                                           \
                        &sht30_i2c_data_##inst,                         \
                        &sht30_i2c_cfg_##inst,                          \
                        POST_KERNEL,                                    \
                        80,                                             \
                        &sht30_api_funcs);                              \

DT_INST_FOREACH_STATUS_OKAY(SHT30_I2C_INIT)
