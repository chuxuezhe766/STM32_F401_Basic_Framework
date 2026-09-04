#define DT_DRV_COMPAT tdk_mpu6050

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "mpu6050.h"

LOG_MODULE_REGISTER(mpu6050, LOG_LEVEL_INF);

/* 寄存器地址 */
#define MPU6050_WHO_AM_I        0x75     /* 器件ID寄存器,期望读到0x68(A0接GND时) */
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_SMPLRT_DIV      0x19     /* 采样率分频寄存器 */
#define MPU6050_CONFIG          0x1A     /* DLPF配置寄存器 */
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_GYRO_OUT        0x43     /* 陀螺仪数据起始地址,连续6字节 */
#define MPU6050_ACC_OUT         0x3B     /* 加速度数据起始地址,连续6字节 */
#define MPU6050_TEMP_OUT        0X41     /* 温度数据起始地址 */
/* 配置值 */
#define MPU6050_WHO_AM_I_ID     0x68
#define MPU6050_GYRO_RANGE      0x18     /* FS_SEL=11, ±2000°/s, 16.4 LSB/(°/s) */
#define MPU6050_ACCEL_RANGE     0x18     /* AFS_SEL=11, ±16g, 2048 LSB/g */
#define MPU6050_SMPLRT_DIV_VAL  0x09     /* DLPF开启时输出1kHz, 1k/(1+9)=100Hz */
#define MPU6050_DLPF_CFG        0x06     /* DLPF=6, 带宽5Hz, 输出率1kHz */

// Configure
struct mpu6050_i2c_config {
    struct i2c_dt_spec i2c;
};

// Data
struct mpu6050_i2c_data {
    struct k_mutex lock;
    uint8_t id;
};

static int mpu6050_write_reg(const struct mpu6050_i2c_config *cfg,
                             uint8_t reg, uint8_t val)
{
    int ret = i2c_reg_write_byte_dt(&cfg->i2c, reg, val);

    if (ret) {
        LOG_ERR("write reg 0x%02x = 0x%02x failed: %d", reg, val, ret);
    }
    return ret;
}

static int mpu6050_read_data(const struct mpu6050_i2c_config *cfg,
                             uint8_t reg, uint8_t *read, size_t len)
{
    int ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, read, len);
    if (ret) {
        LOG_ERR("read reg 0x%02x len %zu failed: %d", reg, len, ret);
    }
    return ret;
}

static int mpu6050_read_id(const struct device *dev, uint8_t *id)
{
    const struct mpu6050_i2c_config *cfg = dev->config;
    struct mpu6050_i2c_data *data = dev->data;
    uint8_t rx;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = mpu6050_read_data(cfg, MPU6050_WHO_AM_I, &rx, 1);
    k_mutex_unlock(&data->lock);
    if (ret) {
        LOG_ERR("read WHO_AM_I failed: %d", ret);
        return ret;
    }
    *id = rx;
    return 0;
}

static int mpu6050_read_acc(const struct device *dev, int16_t *accData)
{
    const struct mpu6050_i2c_config *cfg = dev->config;
    struct mpu6050_i2c_data *data = dev->data;
    uint8_t buf[6];
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = mpu6050_read_data(cfg, MPU6050_ACC_OUT, buf, 6);
    k_mutex_unlock(&data->lock);
    if (ret) {
        LOG_ERR("read acc failed: %d", ret);
        return ret;
    }
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
    return 0;
}

static int mpu6050_read_gyro(const struct device *dev, int16_t *gyroData)
{
    const struct mpu6050_i2c_config *cfg = dev->config;
    struct mpu6050_i2c_data *data = dev->data;
    uint8_t buf[6];
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = mpu6050_read_data(cfg, MPU6050_GYRO_OUT, buf, 6);
    k_mutex_unlock(&data->lock);
    if (ret) {
        LOG_ERR("read gyro failed: %d", ret);
        return ret;
    }
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
    return 0;
}

static int mpu6050_read_temp(const struct device *dev, int16_t *tempData)
{
    const struct mpu6050_i2c_config *cfg = dev->config;
    struct mpu6050_i2c_data *data = dev->data;
    uint8_t buf[2];
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = mpu6050_read_data(cfg, MPU6050_TEMP_OUT, buf, 2);
    k_mutex_unlock(&data->lock);
    if (ret) {
        LOG_ERR("read temp failed: %d", ret);
        return ret;
    }
    *tempData = (buf[0] << 8) | buf[1];
    return 0;
}

static int mpu6050_read_all(const struct device *dev, int16_t *accData,
                        int16_t *gyroData, int16_t *tempData)
{
    const struct mpu6050_i2c_config *cfg = dev->config;
    struct mpu6050_i2c_data *data = dev->data;
    uint8_t buf[14];
    int ret;
    k_mutex_lock(&data->lock, K_FOREVER);
    ret = mpu6050_read_data(cfg, MPU6050_ACC_OUT, buf, 14);
    k_mutex_unlock(&data->lock);
    if (ret) {
        LOG_ERR("read all data failed: %d", ret);
        return ret;
    }
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
    *tempData = (buf[6] << 8) | buf[7];
    gyroData[0] = (buf[8] << 8) | buf[9];
    gyroData[1] = (buf[10] << 8) | buf[11];
    gyroData[2] = (buf[12] << 8) | buf[13];    
    return 0;
}

// Initialize mpu6050
static int mpu6050_init(const struct device *dev)
{
    const struct mpu6050_i2c_config *cfg = dev->config;
    struct mpu6050_i2c_data *data = dev->data;
    uint8_t id;
    int ret;

    // Check I2C
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    // Init mutex
    k_mutex_init(&data->lock);

    k_msleep(100);

    // 软件复位,DEVICE_RESET位完成后自动清零,期间寄存器回默认值
    ret = mpu6050_write_reg(cfg, MPU6050_PWR_MGMT_1, 0x80);
    if (ret) {
        return ret;
    }
    k_msleep(100);

    // 退出休眠,时钟源选X轴陀螺仪PLL(比内部8MHz RC振荡器准)
    ret = mpu6050_write_reg(cfg, MPU6050_PWR_MGMT_1, 0x01);
    if (ret) {
        return ret;
    }

    // 采样率100Hz
    ret = mpu6050_write_reg(cfg, MPU6050_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);
    if (ret) {
        return ret;
    }

    // DLPF=6,加速度/陀螺仪带宽5Hz,数据平滑但延迟约19ms
    ret = mpu6050_write_reg(cfg, MPU6050_CONFIG, MPU6050_DLPF_CFG);
    if (ret) {
        return ret;
    }

    ret = mpu6050_write_reg(cfg, MPU6050_GYRO_CONFIG, MPU6050_GYRO_RANGE);
    if (ret) {
        return ret;
    }

    ret = mpu6050_write_reg(cfg, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE);
    if (ret) {
        return ret;
    }

    // 六轴全部使能,不待机
    ret = mpu6050_write_reg(cfg, MPU6050_PWR_MGMT_2, 0x00);
    if (ret) {
        return ret;
    }

    ret = mpu6050_read_id(dev, &id);
    if (ret) {
        return ret;
    }
    if (id != MPU6050_WHO_AM_I_ID) {
        LOG_ERR("unexpected WHO_AM_I 0x%02x (expected 0x%02x)",
                id, MPU6050_WHO_AM_I_ID);
        return -EINVAL;
    }
    data->id = id;

    LOG_INF("MPU6050 detected, WHO_AM_I = 0x%02x", id);

    LOG_INF("MPU6050 init done");
    return 0;
}

// Define the public API functions for the driver
static const struct mpu6050_api mpu6050_api_funcs = {
    .read_acc = mpu6050_read_acc,
    .read_gyro = mpu6050_read_gyro,
    .read_temp = mpu6050_read_temp,
    .read_all = mpu6050_read_all,
};

// Instance boilerplate

#define MPU6050_I2C_INIT(inst)                                              \
    static const struct mpu6050_i2c_config mpu6050_i2c_cfg_##inst = {      \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
    };                                                                     \
    static struct mpu6050_i2c_data mpu6050_i2c_data_##inst;                \
                                                                           \
    DEVICE_DT_INST_DEFINE(inst,                                            \
                        mpu6050_init,                                      \
                        NULL,                                              \
                        &mpu6050_i2c_data_##inst,                          \
                        &mpu6050_i2c_cfg_##inst,                           \
                        POST_KERNEL,                                       \
                        80,                                                \
                        &mpu6050_api_funcs);                               \

DT_INST_FOREACH_STATUS_OKAY(MPU6050_I2C_INIT)
