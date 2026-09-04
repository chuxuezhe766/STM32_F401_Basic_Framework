#ifndef __ZEPHYR_DRIVERS_W25Q128_H__
#define __ZEPHYR_DRIVERS_W25Q128_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

// Define W25Q128 API
struct w25q128_api {
    int (*write)(const struct device *dev, uint32_t addr,
                            uint8_t *buf, size_t len);
    int (*write_noe)(const struct device *dev, uint32_t addr,
                            uint8_t *buf, size_t len);
    /* 按 4KB 扇区擦除,addr 和 len 都须按扇区对齐 */
    int (*erase)(const struct device *dev, uint32_t addr, size_t len);
    int (*read)(const struct device *dev, uint32_t addr,
                            uint8_t *buf, size_t len);
};

static inline int flash_read(const struct device *dev, uint32_t addr,
                            uint8_t *buf, size_t len)
{
    const struct w25q128_api *api = dev->api;
    if (api == NULL || api->read == NULL) {
        return -ENOSYS;
    }
    return api->read(dev, addr, buf, len);
}

static inline int flash_write(const struct device *dev, uint32_t addr,
                            uint8_t *buf, size_t len)
{
    const struct w25q128_api *api = dev->api;
    if (api == NULL || api->write == NULL) {
        return -ENOSYS;
    }
    return api->write(dev, addr, buf, len);
}

static inline int flash_write_noe(const struct device *dev, uint32_t addr,
                                  uint8_t *buf, size_t len)
{
    const struct w25q128_api *api = dev->api;
    if (api == NULL || api->write_noe == NULL) {
        return -ENOSYS;
    }
    return api->write_noe(dev, addr, buf, len);
}

static inline int flash_erase(const struct device *dev, uint32_t addr,
                              size_t len)
{
    const struct w25q128_api *api = dev->api;
    if (api == NULL || api->erase == NULL) {
        return -ENOSYS;
    }
    return api->erase(dev, addr, len);
}

#endif
