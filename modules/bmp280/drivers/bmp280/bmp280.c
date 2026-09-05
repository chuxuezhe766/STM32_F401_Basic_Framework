#define DT_DRV_COMPAT bosch_w25q128

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "bmp280.h"

LOG_MODULE_REGISTER(bmp280, LOG_LEVEL_INF);

// Configure
struct bmp280_i2c_config {
    struct i2c_dt_spec i2c;
};

// Data
struct bmp280_i2c_data {
    struct k_mutex lock;
    uint8_t id;
}

