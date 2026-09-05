#ifndef __ZEPHYR_DRIVERS_SHT30_H__
#define __ZEPHYR_DRIVERS_SHT30_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

/*
 * Generic temperature + humidity interface.
 *
 * Public functions use the th_ prefix (same idea as gyro_ in
 * mpu6050.h, mag_ in qmc5883l.h and flash_ in w25q128.h) and never
 * expose the chip model: swapping SHT30 for another temp/humidity
 * sensor (SHT31/SHT40/...) only requires the new driver to implement
 * the same api struct. Neither the app nor any other code needs any
 * change.
 */

// Define SHT30 API
struct sht30_api {
    /* One single-shot measurement, high repeatability. Blocks for
     * the conversion time (~15ms). Outputs are already converted:
     * temp_c in degrees Celsius, hum_pct in %RH (0..100) - the SHT30
     * scaling is fixed by the datasheet, there is no full-scale
     * register to configure. */
    int (*read_th)(const struct device *dev, float *temp_c, float *hum_pct);
    /* Read the raw 16-bit status register */
    int (*read_status)(const struct device *dev, uint16_t *status);
    /* Soft reset, ~1.5ms */
    int (*reset)(const struct device *dev);
};

static inline int th_get_all(const struct device *dev,
                             float *temp_c, float *hum_pct)
{
    const struct sht30_api *api = dev->api;
    if (api == NULL || api->read_th == NULL) {
        return -ENOSYS;
    }
    return api->read_th(dev, temp_c, hum_pct);
}

static inline int th_get_status(const struct device *dev, uint16_t *status)
{
    const struct sht30_api *api = dev->api;
    if (api == NULL || api->read_status == NULL) {
        return -ENOSYS;
    }
    return api->read_status(dev, status);
}

static inline int th_reset(const struct device *dev)
{
    const struct sht30_api *api = dev->api;
    if (api == NULL || api->reset == NULL) {
        return -ENOSYS;
    }
    return api->reset(dev);
}

#endif
