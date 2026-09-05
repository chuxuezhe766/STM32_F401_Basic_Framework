#ifndef __ZEPHYR_DRIVERS_MPU6050_H__
#define __ZEPHYR_DRIVERS_MPU6050_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

/*
 * Generic gyro (6-axis IMU) interface.
 *
 * Public functions use the gyro_ prefix (same idea as flash_ in
 * w25q128.h) and never expose the chip model: the upper layers only
 * depend on this API, so swapping MPU6050 for a BMI088/ICM-42688
 * only requires the new driver to implement the same api struct.
 * Neither the app nor the attitude code needs any change.
 */

// Define MPU6050 API
struct mpu6050_api {
    int (*read_acc)(const struct device *dev, int16_t *accData);
    int (*read_gyro)(const struct device *dev, int16_t *gyroData);
    int (*read_temp)(const struct device *dev, int16_t *tempData);
    /* Read accel + temp + gyro in one transaction, all from the
     * same sample instant */
    int (*read_all)(const struct device *dev, int16_t *accData,
                    int16_t *gyroData, int16_t *tempData);
};

static inline int gyro_get_acc(const struct device *dev, int16_t *accData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_acc == NULL) {
        return -ENOSYS;
    }
    return api->read_acc(dev, accData);
}

static inline int gyro_get_gyro(const struct device *dev, int16_t *gyroData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_gyro == NULL) {
        return -ENOSYS;
    }
    return api->read_gyro(dev, gyroData);
}

static inline int gyro_get_temp(const struct device *dev, int16_t *tempData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_temp == NULL) {
        return -ENOSYS;
    }
    return api->read_temp(dev, tempData);
}

static inline int gyro_get_all(const struct device *dev, int16_t *accData,
                               int16_t *gyroData, int16_t *tempData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_all == NULL) {
        return -ENOSYS;
    }
    return api->read_all(dev, accData, gyroData, tempData);
}

#endif
