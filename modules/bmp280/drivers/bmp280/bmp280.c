#define DT_DRV_COMPAT bosch_bmp280

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "bmp280.h"

LOG_MODULE_REGISTER(bmp280, LOG_LEVEL_INF);

/* The registers of bmp280 */
#define BMP280_REG_ID           0xD0    /* chip id register */
#define BMP280_REG_RESET        0xE0    /* write 0xB6 to soft reset */
#define BMP280_REG_STATUS       0xF3    /* bit3: measuring, bit0: im_update */
#define BMP280_REG_CTRL_MEAS    0xF4    /* osrs_t | osrs_p | mode */
#define BMP280_REG_CONFIG       0xF5    /* t_sb | filter (normal mode only) */
#define BMP280_REG_PRESS_MSB    0xF7    /* data start: burst-read 6 bytes,
                                           press[19:0] then temp[19:0], MSB first */
#define BMP280_REG_CALIB        0x88    /* factory calibration, 26 bytes,
                                           dig_T1..dig_P9, LSB first */

/* The values of bmp280 */
#define BMP280_CHIP_ID          0x58    /* expected chip id, BME280 is 0x60 */
#define BMP280_RESET_CMD        0xB6
#define BMP280_STATUS_MEASURING BIT(3)

/* ctrl_meas bit layout: [7:5]=osrs_t [4:2]=osrs_p [1:0]=mode */
#define BMP280_OSRS_T(x)    ((x) << 5)
#define BMP280_OSRS_P(x)    ((x) << 2)
#define BMP280_MODE_SLEEP   0x00
#define BMP280_MODE_FORCED  0x01

/* Weather monitoring setup (datasheet table 7): forced mode,
 * osrs_t = x1, osrs_p = x1. Forced measures once per write and
 * returns to sleep - simplest model for a poll-style driver. */
#define BMP280_CTRL_MEAS_VAL \
    (BMP280_OSRS_T(0x01) | BMP280_OSRS_P(0x01) | BMP280_MODE_FORCED)

/* Max conversion time at x1/x1 oversampling (datasheet 3.8.3,
 * t_measure_max = 6ms), plus margin for the status poll */
#define BMP280_MEAS_TIMEOUT_MS  50
#define BMP280_POLL_INTERVAL_MS 2
/* Datasheet: the chip needs ~2ms after soft reset */
#define BMP280_RESET_TIME_MS    2

/* Factory calibration coefficients, datasheet chapter 3.11.2.
 * dig_T1/dig_P1 are unsigned, the rest are signed. */
struct bmp280_calib {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
};

// Configure
struct bmp280_i2c_config {
    struct i2c_dt_spec i2c;
};

// Data
struct bmp280_i2c_data {
    struct k_mutex lock;
    struct bmp280_calib calib;
    /* Fine temperature, shared between the temp and pressure
     * compensation formulas (datasheet 3.11.3) */
    float t_fine;
    uint8_t id;
};

static int bmp280_write_reg(const struct bmp280_i2c_config *cfg,
                             uint8_t reg, uint8_t val)
{
    int ret = i2c_reg_write_byte_dt(&cfg->i2c, reg, val);

    if (ret) {
        LOG_ERR("write reg 0x%02x = 0x%02x failed: %d", reg, val, ret);
    }
    return ret;
}

static int bmp280_read_data(const struct bmp280_i2c_config *cfg,
                            uint8_t reg, uint8_t *read, size_t len)
{
    int ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, read, len);
    if (ret) {
        LOG_ERR("read reg 0x%02x len %zu failed: %d", reg, len, ret);
    }
    return ret;
}

static int bmp280_read_id(const struct device *dev, uint8_t *id)
{
    const struct bmp280_i2c_config *cfg = dev->config;
    struct bmp280_i2c_data *data = dev->data;
    uint8_t rx;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = bmp280_read_data(cfg, BMP280_REG_ID, &rx, 1);
    k_mutex_unlock(&data->lock);
    if (ret == 0) {
        *id = rx;
    }
    return ret;
}

/* Read the 26 factory calibration bytes once at init: the raw ADC
 * values are useless without them. All coefficients are LSB first,
 * like every 16-bit value in this chip. */
static int bmp280_read_calib(const struct device *dev)
{
    const struct bmp280_i2c_config *cfg = dev->config;
    struct bmp280_i2c_data *data = dev->data;
    uint8_t buf[26];
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = bmp280_read_data(cfg, BMP280_REG_CALIB, buf, sizeof(buf));
    k_mutex_unlock(&data->lock);
    if (ret) {
        return ret;
    }

    data->calib.dig_T1 = (uint16_t)(buf[1]  << 8) | buf[0];
    data->calib.dig_T2 = (int16_t)((uint16_t)(buf[3]  << 8) | buf[2]);
    data->calib.dig_T3 = (int16_t)((uint16_t)(buf[5]  << 8) | buf[4]);
    data->calib.dig_P1 = (uint16_t)(buf[7]  << 8) | buf[6];
    data->calib.dig_P2 = (int16_t)((uint16_t)(buf[9]  << 8) | buf[8]);
    data->calib.dig_P3 = (int16_t)((uint16_t)(buf[11] << 8) | buf[10]);
    data->calib.dig_P4 = (int16_t)((uint16_t)(buf[13] << 8) | buf[12]);
    data->calib.dig_P5 = (int16_t)((uint16_t)(buf[15] << 8) | buf[14]);
    data->calib.dig_P6 = (int16_t)((uint16_t)(buf[17] << 8) | buf[16]);
    data->calib.dig_P7 = (int16_t)((uint16_t)(buf[19] << 8) | buf[18]);
    data->calib.dig_P8 = (int16_t)((uint16_t)(buf[21] << 8) | buf[20]);
    data->calib.dig_P9 = (int16_t)((uint16_t)(buf[23] << 8) | buf[22]);
    return 0;
}

/* Temperature compensation, datasheet 3.11.3 (the float variant of
 * the reference code). Updates t_fine as a side effect: the pressure
 * formula needs it, so temperature is ALWAYS compensated first. */
static float bmp280_compensate_temp(struct bmp280_i2c_data *data,
                                    int32_t adc_T)
{
    const struct bmp280_calib *c = &data->calib;
    float var1, var2;

    var1 = ((float)adc_T) / 16384.0f - ((float)c->dig_T1) / 1024.0f;
    var1 = var1 * (float)c->dig_T2;
    var2 = ((float)adc_T) / 131072.0f - ((float)c->dig_T1) / 8192.0f;
    var2 = var2 * var2 * (float)c->dig_T3;

    data->t_fine = var1 + var2;
    return (var1 + var2) / 5120.0f;
}

/* Pressure compensation, same source. Uses t_fine from the
 * temperature pass, so call order matters. */
static float bmp280_compensate_press(const struct bmp280_i2c_data *data,
                                     int32_t adc_P)
{
    const struct bmp280_calib *c = &data->calib;
    float var1, var2, p;

    var1 = data->t_fine / 2.0f - 64000.0f;
    var2 = var1 * var1 * (float)c->dig_P6 / 32768.0f;
    var2 = var2 + var1 * (float)c->dig_P5 * 2.0f;
    var2 = var2 / 4.0f + (float)c->dig_P4 * 65536.0f;
    var1 = ((float)c->dig_P3 * var1 * var1 / 524288.0f
            + (float)c->dig_P2 * var1) / 52736.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)c->dig_P1;

    p = 1048576.0f - (float)adc_P;
    p = (p - var2 / 4096.0f) * 6250.0f / var1;
    var1 = (float)c->dig_P9 * p * p / 2147483648.0f;
    var2 = p * (float)c->dig_P8 / 32768.0f;
    p = p + (var1 + var2 + (float)c->dig_P7) / 16.0f;
    return p;
}

/* Trigger one forced-mode measurement, wait for it to finish, then
 * burst-read press[19:0] + temp[19:0] from 0xF7 in one transaction. */
static int bmp280_measure_raw(const struct bmp280_i2c_config *cfg,
                              uint8_t buf[6])
{
    int64_t deadline;
    uint8_t status;
    int ret;

    /* Forced mode measures once per write: every read needs a fresh
     * trigger, after which the chip goes back to sleep by itself */
    ret = bmp280_write_reg(cfg, BMP280_REG_CTRL_MEAS,
                           BMP280_CTRL_MEAS_VAL);
    if (ret) {
        return ret;
    }

    /* Poll the measuring flag (bit3 of status 0xF3) instead of a
     * blind delay: the flag clears exactly when the result is ready */
    deadline = k_uptime_get() + BMP280_MEAS_TIMEOUT_MS;
    do {
        k_msleep(BMP280_POLL_INTERVAL_MS);
        ret = bmp280_read_data(cfg, BMP280_REG_STATUS, &status, 1);
        if (ret) {
            return ret;
        }
        if (!(status & BMP280_STATUS_MEASURING)) {
            break;
        }
        if (k_uptime_get() >= deadline) {
            LOG_ERR("measuring timeout, status 0x%02x", status);
            return -ETIMEDOUT;
        }
    } while (1);

    return bmp280_read_data(cfg, BMP280_REG_PRESS_MSB, buf, 6);
}

static int bmp280_read_all(const struct device *dev, float *press_pa,
                           float *temp_c)
{
    const struct bmp280_i2c_config *cfg = dev->config;
    struct bmp280_i2c_data *data = dev->data;
    uint8_t buf[6];
    int32_t adc_P, adc_T;
    int ret;

    if (press_pa == NULL || temp_c == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    ret = bmp280_measure_raw(cfg, buf);
    if (ret == 0) {
        /* Two 20-bit values packed in 6 bytes, MSB first */
        adc_P = (int32_t)(((uint32_t)buf[0] << 12) |
                          ((uint32_t)buf[1] << 4) |
                          ((uint32_t)buf[2] >> 4));
        adc_T = (int32_t)(((uint32_t)buf[3] << 12) |
                          ((uint32_t)buf[4] << 4) |
                          ((uint32_t)buf[5] >> 4));
        *temp_c = bmp280_compensate_temp(data, adc_T);
        *press_pa = bmp280_compensate_press(data, adc_P);
    }

    k_mutex_unlock(&data->lock);
    return ret;
}

static int bmp280_reset(const struct device *dev)
{
    const struct bmp280_i2c_config *cfg = dev->config;
    struct bmp280_i2c_data *data = dev->data;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = bmp280_write_reg(cfg, BMP280_REG_RESET, BMP280_RESET_CMD);
    k_mutex_unlock(&data->lock);
    if (ret == 0) {
        k_msleep(BMP280_RESET_TIME_MS);
    }
    return ret;
}

// Initialize bmp280
static int bmp280_init(const struct device *dev)
{
    const struct bmp280_i2c_config *cfg = dev->config;
    struct bmp280_i2c_data *data = dev->data;
    uint8_t id;
    int ret;

    // Check i2c
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    // Init mutex
    k_mutex_init(&data->lock);

    // Soft reset: all registers back to default, ~2ms
    ret = bmp280_reset(dev);
    if (ret) {
        return ret;
    }

    // Read id: warn only on mismatch, BME280 (0x60) also works for
    // pressure/temperature with the same formulas
    ret = bmp280_read_id(dev, &id);
    if (ret) {
        return ret;
    }
    if (id != BMP280_CHIP_ID) {
        LOG_WRN("unexpected ID 0x%02x (expected 0x%02x), continuing",
                id, BMP280_CHIP_ID);
    } else {
        LOG_INF("BMP280 detected, ID = 0x%02x", id);
    }
    data->id = id;

    // Load the factory calibration coefficients
    ret = bmp280_read_calib(dev);
    if (ret) {
        return ret;
    }

    LOG_INF("BMP280 init done");
    return 0;
}

// Define the public API functions for the driver
static const struct bmp280_api bmp280_api_funcs = {
    .read_all = bmp280_read_all,
    .reset = bmp280_reset,
};

// Instance boilerplate

#define BMP280_I2C_INIT(inst)                                           \
    static const struct bmp280_i2c_config bmp280_i2c_cfg_##inst = {     \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                              \
    };                                                                  \
    static struct bmp280_i2c_data bmp280_i2c_data_##inst;               \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                        bmp280_init,                                    \
                        NULL,                                           \
                        &bmp280_i2c_data_##inst,                        \
                        &bmp280_i2c_cfg_##inst,                         \
                        POST_KERNEL,                                    \
                        80,                                             \
                        &bmp280_api_funcs);                             \

DT_INST_FOREACH_STATUS_OKAY(BMP280_I2C_INIT)
