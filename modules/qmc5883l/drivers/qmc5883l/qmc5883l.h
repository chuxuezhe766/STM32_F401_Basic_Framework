#ifndef __ZEPHYR_DRIVERS_QMC5883L_H__
#define __ZEPHYR_DRIVERS_QMC5883L_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

/*
 * Generic magnetometer interface.
 *
 * Public functions use the mag_ prefix (same idea as flash_ in
 * w25q128.h and gyro_ in mpu6050.h) and never expose the chip model:
 * swapping QMC5883L for another compass only requires the new driver
 * to implement the same api struct. Neither the app nor the attitude
 * code needs any change.
 */

// Define QMC5883L API
struct qmc5883l_api {
    /* Read one magnetic field sample, int16_t mag[3] = {x, y, z},
     * raw LSB. Full scale +/-8Gs -> 3000 LSB/Gs, +/-2Gs -> 12000.
     * Blocks until the DRDY flag or timeout (~50ms max). */
    int (*read_mag)(const struct device *dev, int16_t mag[3]);
};

static inline int mag_get_all(const struct device *dev, int16_t mag[3])
{
    const struct qmc5883l_api *api = dev->api;
    if (api == NULL || api->read_mag == NULL) {
        return -ENOSYS;
    }
    return api->read_mag(dev, mag);
}

#endif
