#ifndef __ZEPHYR_DRIVERS_BMP280_H__
#define __ZEPHYR_DRIVERS_BMP280_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

/*
 * Generic barometer (pressure + temperature) interface.
 *
 * Public functions use the baro_ prefix (same idea as gyro_ in
 * mpu6050.h, mag_ in qmc5883l.h, th_ in sht30.h and flash_ in
 * w25q128.h) and never expose the chip model: swapping BMP280 for
 * another pressure sensor (BMP388/DPS310/...) only requires the new
 * driver to implement the same api struct. Neither the app nor any
 * other code needs any change.
 */

// Define BMP280 API
struct bmp280_api {
    /* One forced-mode measurement. Outputs are already converted to
     * physical units: press_pa in Pascal, temp_c in degrees Celsius.
     * The temperature is measured anyway - the pressure compensation
     * formula needs it (t_fine), so there is no read_temp only. */
    int (*read_all)(const struct device *dev, float *press_pa,
                    float *temp_c);
    /* Soft reset, all registers back to default, ~2ms */
    int (*reset)(const struct device *dev);
};

static inline int baro_get_all(const struct device *dev,
                               float *press_pa, float *temp_c)
{
    const struct bmp280_api *api = dev->api;
    if (api == NULL || api->read_all == NULL) {
        return -ENOSYS;
    }
    return api->read_all(dev, press_pa, temp_c);
}

static inline int baro_reset(const struct device *dev)
{
    const struct bmp280_api *api = dev->api;
    if (api == NULL || api->reset == NULL) {
        return -ENOSYS;
    }
    return api->reset(dev);
}

#endif
