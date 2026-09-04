#ifndef _ZEPHYR_DRIVERS_MPU6050__H__
#define _ZEPHYR_DRIVERS_MPU6050__H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

// Define MPU6050 API
struct mpu6050_api {
    int (*read_acc)(const struct device *dev, int16_t *accData);
    int (*read_gyro)(const struct device *dev, int16_t *gyroData);
    int (*read_temp)(const struct device *dev, int16_t *tempData);
    /* 一次事务读回加速度+温度+陀螺仪,三组数据来自同一采样瞬间 */
    int (*read_all)(const struct device *dev, int16_t *accData,
                    int16_t *gyroData, int16_t *tempData);
};

static inline int mpu6050_get_acc(const struct device *dev, int16_t *accData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_acc == NULL) {
        return -ENOSYS;
    }
    return api->read_acc(dev, accData);
}

static inline int mpu6050_get_gyro(const struct device *dev, int16_t *gyroData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_gyro == NULL) {
        return -ENOSYS;
    }
    return api->read_gyro(dev, gyroData);
}

static inline int mpu6050_get_temp(const struct device *dev, int16_t *tempData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_temp == NULL) {
        return -ENOSYS;
    }
    return api->read_temp(dev, tempData);
}

static inline int mpu6050_get_all(const struct device *dev, int16_t *accData,
                                  int16_t *gyroData, int16_t *tempData)
{
    const struct mpu6050_api *api = dev->api;
    if (api == NULL || api->read_all == NULL) {
        return -ENOSYS;
    }
    return api->read_all(dev, accData, gyroData, tempData);
}

#endif
