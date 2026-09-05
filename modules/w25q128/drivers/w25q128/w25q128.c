#define DT_DRV_COMPAT winbond_w25q128

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "w25q128.h"

LOG_MODULE_REGISTER(w25q128, LOG_LEVEL_INF);

// W25q128_macro
#define W25Q128_JEDEC_ID    0xEF4018     /* EF 40 18 */
#define W25Q128_CHIP_ID     0xEF17       /* EF 17 */
#define CMD_READ_JEDEC_ID   0x9F        // Read jedec id
#define CMD_READ_STATE      0X05        // Read state
#define CMD_READ_CHIP_ID    0x90        // Read chip id
#define CMD_READ_DATA       0x03        // Read data
#define CMD_FAST_READ_DATA  0x0B        // Fast Read data
#define CMD_ERASE_CHIP      0xC7        // Erase chip
#define CMD_ERASE_SECTOR    0x20        // Erase sector(4KB)
#define CMD_ERASE_BLOCK     0xD8        // Erase block(64KB)  
#define CMD_WRITE_ENABLE    0x06        // Write Enable
#define CMD_WRITE_PAGE      0x02        // Write PAGE
#define W25Q128_SIZE_BYTES  (DT_PROP(DT_NODELABEL(w25q128), size) / 8)
#define W25Q128_SECTOR_SIZE 4096u         // 扇区大小(4KB)
#define W25Q128_PAGE_SIZE   256u          // 页大小(256B)
#define W25Q128_SR1_BUSY    BIT(0)        // SR1 bit0: 1=忙(写/擦进行中)

// Configure
struct w25q128_spi_config {
    struct spi_dt_spec spi;
};

struct w25q128_spi_data {
    struct k_mutex lock;
    uint8_t flash_state;
    uint16_t flash_chip_id;
	uint32_t flash_jedec_id;
};

// Split the 24-bit address into three bytes
static void w25q128_splitaddr(uint32_t addr, uint8_t *addrHigh, uint8_t *addrMid, uint8_t *addrLow)
{
    *addrHigh = (addr >> 16);
    addr = addr & 0x0000FFFF;       // Mask the high byte
    *addrMid = (addr >> 8);         
    *addrLow = addr & 0x000000FF;   // Make the mid byte
}

// Read jedec id
static int w25q128_read_jedec_id(const struct device *dev, uint32_t *id)
{
    const struct w25q128_spi_config *cfg = dev->config;
    uint8_t tx[4] = {CMD_READ_JEDEC_ID, 0, 0, 0};
    uint8_t rx[4] = {0};
    
    const struct spi_buf tx_buf = {
        .buf = tx,
        .len = sizeof(tx)
    };
    const struct spi_buf rx_buf = {
        .buf = rx,
        .len = sizeof(rx)
    };
    const struct spi_buf_set txs = {
        .buffers = &tx_buf,
        .count = 1
    };
    const struct spi_buf_set rxs = {
        .buffers = &rx_buf,
        .count = 1
    };

    int ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) {
        return ret;
    }
    *id = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
    return 0;
}

// Read chip id
static int w25q128_read_chip_id(const struct device *dev, uint16_t *id)
{
    const struct w25q128_spi_config *cfg = dev->config;
    uint8_t tx[4] = {CMD_READ_CHIP_ID, 0, 0, 0};
    uint8_t rx[6] = {0};
    
    const struct spi_buf tx_buf = {
        .buf = tx,
        .len = sizeof(tx)
    };
    const struct spi_buf rx_buf = {
        .buf = rx,
        .len = sizeof(rx)
    };
    const struct spi_buf_set txs = {
        .buffers = &tx_buf,
        .count = 1
    };
    const struct spi_buf_set rxs = {
        .buffers = &rx_buf,
        .count = 1
    };

    int ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) {
        return ret;
    }
    *id = ((uint16_t)rx[4] << 8) | rx[5];
    return 0;
}

// Read flash status
static int w25q128_read_state(const struct device *dev, uint8_t *state)
{
    const struct w25q128_spi_config *cfg = dev->config;
    uint8_t tx[2] = {CMD_READ_STATE, 0};
    uint8_t rx[2] = {0};
    int ret;

    const struct spi_buf tx_buf = {
        .buf = tx,
        .len = sizeof(tx)
    };
    const struct spi_buf rx_buf = {
        .buf = rx,
        .len = sizeof(rx)
    };
    const struct spi_buf_set txs = {
        .buffers = &tx_buf,
        .count = 1
    };
    const struct spi_buf_set rxs = {
        .buffers = &rx_buf,
        .count = 1
    };
    ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) {
        return ret;
    }
    *state = rx[1];
    return 0;
}

// Wait busy
static int w25q128_wait_busy(const struct device *dev, uint32_t timeout_ms)
{
    uint8_t state;
    int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;
    int ret;

    do {
        ret = w25q128_read_state(dev, &state);
        if (ret) {
            return ret;
        }
        if ((state & W25Q128_SR1_BUSY) == 0) {
            return 0;
        }
        k_msleep(1);
    } while (k_uptime_get() < deadline);
    return -ETIMEDOUT;
}

// Read from addr to buf
static int w25q128_read(const struct device *dev, uint32_t addr, uint8_t *buf, size_t len)
{
    const struct w25q128_spi_config *cfg = dev->config;
    uint8_t byte2, byte3, byte4;
    int ret;

    w25q128_splitaddr(addr, &byte2, &byte3, &byte4);
    // Check buf and len
    if (buf == NULL || len == 0) {
        return -EINVAL;
    }
    // Check addr
    if (addr >= W25Q128_SIZE_BYTES) {
        return -ERANGE;
    }

    uint8_t tx[4] = {CMD_READ_DATA, byte2, byte3, byte4};

    const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf rx_bufs[] = {{.buf = NULL, .len = sizeof(tx)},
                        {.buf = buf, .len = len}};
    const struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rxs = {.buffers = rx_bufs, .count = 2};

    ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) {
        return ret;
    }
    return 0;
}

// Fast Read from addr to buf
static int w25q128_fastread(const struct device *dev, uint32_t addr, uint8_t *buf, size_t len)
{
    const struct w25q128_spi_config *cfg = dev->config;
    uint8_t byte2, byte3, byte4;
    int ret;

    w25q128_splitaddr(addr, &byte2, &byte3, &byte4);
    // Check buf and len
    if (buf == NULL || len == 0) {
        return -EINVAL;
    }
    // Check addr
    if (addr >= W25Q128_SIZE_BYTES) {
        return -ERANGE;
    }

    uint8_t tx[5] = {CMD_FAST_READ_DATA, byte2, byte3, byte4, 0};

    const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf rx_bufs[] = {{.buf = NULL, .len = sizeof(tx)},
                        {.buf = buf, .len = len}};
    const struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rxs = {.buffers = rx_bufs, .count = 2};

    ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) {
        return ret;
    }
    return 0;
}

// Write Enable: 擦除/编程前必须先发 0x06 置位 WEL,否则命令被 Flash 忽略
static int w25q128_write_enable(const struct device *dev)
{
    const struct w25q128_spi_config *cfg = dev->config;
    uint8_t cmd = CMD_WRITE_ENABLE;
    const struct spi_buf tx_buf = {.buf = &cmd, .len = sizeof(cmd)};
    const struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};

    return spi_write_dt(&cfg->spi, &txs);
}

// Erase whole chip
static int w25q128_erasechip(const struct device *dev)
{
    const struct w25q128_spi_config *cfg = dev->config;
    int ret;

    /* 先等之前可能的操作完成 */
    ret = w25q128_wait_busy(dev, 50);
    if (ret) {
        return ret;
    }

    /* 擦除前必须写使能 */
    ret = w25q128_write_enable(dev);
    if (ret) {
        return ret;
    }

    /* 发整片擦除命令(仅 TX,无返回数据) */
    uint8_t cmd = CMD_ERASE_CHIP;
    const struct spi_buf tx_buf = {.buf = &cmd, .len = sizeof(cmd)};
    const struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
    ret = spi_write_dt(&cfg->spi, &txs);
    if (ret) {
        return ret;
    }

    /* 整片擦除耗时长(W25Q128 典型 20s),超时给足 */
    return w25q128_wait_busy(dev, 60000);
}

// Erase sector
static int w25q128_erasesector(const struct device *dev, uint32_t addr)
{
    const struct w25q128_spi_config *cfg = dev->config;
    int ret;
    uint8_t byte2, byte3, byte4;

    w25q128_splitaddr(addr, &byte2, &byte3, &byte4);
    /* 先等之前可能的操作完成 */
    ret = w25q128_wait_busy(dev, 50);
    if (ret) {
        return ret;
    }

    /* 擦除前必须写使能 */
    ret = w25q128_write_enable(dev);
    if (ret) {
        return ret;
    }

    /* 发整片擦除命令(仅 TX,无返回数据) */
    uint8_t tx[4] = {CMD_ERASE_SECTOR, byte2, byte3, byte4};
    const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
    ret = spi_write_dt(&cfg->spi, &txs);
    if (ret) {
        return ret;
    }

    /* W25Q128 扇区擦除典型 30ms,最大 200ms */
    return w25q128_wait_busy(dev, 200);
}

// Erase block
static int w25q128_eraseblock(const struct device *dev, uint32_t addr)
{
    const struct w25q128_spi_config *cfg = dev->config;
    int ret;
    uint8_t byte2, byte3, byte4;

    w25q128_splitaddr(addr, &byte2, &byte3, &byte4);
    /* 先等之前可能的操作完成 */
    ret = w25q128_wait_busy(dev, 50);
    if (ret) {
        return ret;
    }

    /* 擦除前必须写使能 */
    ret = w25q128_write_enable(dev);
    if (ret) {
        return ret;
    }

    /* 发整片擦除命令(仅 TX,无返回数据) */
    uint8_t tx[4] = {CMD_ERASE_BLOCK, byte2, byte3, byte4};
    const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf_set txs = {.buffers = &tx_buf, .count = 1};
    ret = spi_write_dt(&cfg->spi, &txs);
    if (ret) {
        return ret;
    }

    /* W25Q128 64KB 块擦除典型 150ms,最大 2000ms */
    return w25q128_wait_busy(dev, 2000);
}

// Write Page
static int w25q128_writepage(const struct device *dev, uint32_t addr, uint8_t *pBuffer, uint16_t bytecount)
{
    const struct w25q128_spi_config *cfg = dev->config;
    int ret;
    uint8_t byte2, byte3, byte4;
    w25q128_splitaddr(addr, &byte2, &byte3, &byte4);
    if (pBuffer == NULL || bytecount == 0 || bytecount > 256) {
        return -EINVAL;
    }
    if (((addr & 0xFFU) + bytecount) > 256U) {     // 跨页 → 拒绝
        return -EINVAL;
    }

    /* 先等之前可能的操作完成 */
    ret = w25q128_wait_busy(dev, 5);
    if (ret) {
        return ret;
    }

    /* 必须写使能 */
    ret = w25q128_write_enable(dev);
    if (ret) {
        return ret;
    }

    uint8_t tx[4] = {CMD_WRITE_PAGE, byte2, byte3, byte4};
    const struct spi_buf tx_bufs[] = {{.buf = tx, .len = sizeof(tx)},
                                {.buf = pBuffer, .len = bytecount}};
    const struct spi_buf_set txs = {.buffers = tx_bufs, .count = 2};

    ret = spi_write_dt(&cfg->spi, &txs);
    if (ret) {
        return ret;
    }

    /* W25Q128 页编程典型 0.4ms,最大 3ms */
    return w25q128_wait_busy(dev, 20);
}

// write but not erase
static int w25q128_write_noe(const struct device *dev, uint32_t addr,
                         uint8_t *buf, size_t len)
{
    struct w25q128_spi_data *data = dev->data;
    int ret;

    if (buf == NULL) {
        return -EINVAL;
    }
    if (len == 0) {
        return 0;
    }
    if (addr >= W25Q128_SIZE_BYTES) {
        return -ERANGE;
    }
    if (len > W25Q128_SIZE_BYTES - addr) {
        return -ERANGE;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    /*按页切分编程 */
    while (len > 0) {
        size_t page_left = W25Q128_PAGE_SIZE - (addr & (W25Q128_PAGE_SIZE - 1u));
        size_t chunk = (len < page_left) ? len : page_left;

        ret = w25q128_writepage(dev, addr, buf, (uint16_t)chunk);
        if (ret) {
            goto out;
        }
        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    ret = 0;

out:
    k_mutex_unlock(&data->lock);
    return ret;
}

// Erase range: addr/len 都必须按 4KB 扇区对齐,逐扇区擦除
static int w25q128_erase(const struct device *dev, uint32_t addr, size_t len)
{
    struct w25q128_spi_data *data = dev->data;
    int ret;

    if (len == 0) {
        return 0;
    }
    if ((addr % W25Q128_SECTOR_SIZE) != 0 ||
        (len % W25Q128_SECTOR_SIZE) != 0) {
        return -EINVAL;
    }
    if (addr >= W25Q128_SIZE_BYTES || len > W25Q128_SIZE_BYTES - addr) {
        return -ERANGE;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    for (uint32_t off = 0; off < len; off += W25Q128_SECTOR_SIZE) {
        ret = w25q128_erasesector(dev, addr + off);
        if (ret) {
            goto out;
        }
    }
    ret = 0;

out:
    k_mutex_unlock(&data->lock);
    return ret;
}

// 任意地址、任意长度写入:先擦除数据覆盖到的扇区,再按页切分编程。
// 整个"擦+写"在互斥锁保护下原子完成,调用者无需提前擦除。
// 注意:擦除按扇区粒度(4KB),会一并擦掉覆盖扇区里的其他数据。
static int w25q128_write(const struct device *dev, uint32_t addr,
                         uint8_t *buf, size_t len)
{
    struct w25q128_spi_data *data = dev->data;
    int ret;

    if (buf == NULL) {
        return -EINVAL;
    }
    if (len == 0) {
        return 0;
    }
    if (addr >= W25Q128_SIZE_BYTES) {
        return -ERANGE;
    }
    if (len > W25Q128_SIZE_BYTES - addr) {
        return -ERANGE;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    /* 1. 擦除数据 [addr, addr+len) 覆盖到的所有扇区(向下取整到扇区起点) */
    uint32_t erase_addr = addr & ~(W25Q128_SECTOR_SIZE - 1u);
    uint32_t end = addr + len;
    while (erase_addr < end) {
        ret = w25q128_erasesector(dev, erase_addr);
        if (ret) {
            goto out;
        }
        erase_addr += W25Q128_SECTOR_SIZE;
    }

    /* 2. 按页切分编程 */
    while (len > 0) {
        size_t page_left = W25Q128_PAGE_SIZE - (addr & (W25Q128_PAGE_SIZE - 1u));
        size_t chunk = (len < page_left) ? len : page_left;

        ret = w25q128_writepage(dev, addr, buf, (uint16_t)chunk);
        if (ret) {
            goto out;
        }
        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    ret = 0;

out:
    k_mutex_unlock(&data->lock);
    return ret;
}


// Initialize w25q128
static int w25q128_init(const struct device *dev)
{
    const struct w25q128_spi_config *cfg = dev->config;
    struct w25q128_spi_data *data = dev->data;
    int ret;

    // Check SPI
    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI not ready");
        return -ENODEV;
    }

    // Init mutex
    k_mutex_init(&data->lock);

    // Read 
    uint32_t jedec_id = 0;
    uint8_t state = 0;
    uint16_t chip_id = 0;
    ret = w25q128_read_jedec_id(dev, &jedec_id);
    if (ret < 0) {
        LOG_ERR("ret jedec id failed: %d", ret);
        return ret;
    }
    data->flash_jedec_id = jedec_id;

    if (data->flash_jedec_id != W25Q128_JEDEC_ID) {
        LOG_WRN("unexpected id 0x%06X (expect 0x%06X)", data->flash_jedec_id, W25Q128_JEDEC_ID);
    }
    else {
        LOG_INF("w25q128 ready, id = 0x%06X", data->flash_jedec_id);
    }

    ret = w25q128_read_chip_id(dev, &chip_id);
    if (ret < 0) {
        LOG_ERR("ret chip id failed: %d", ret);
        return ret;
    }
    data->flash_chip_id = chip_id;

    if (data->flash_chip_id != W25Q128_CHIP_ID) {
        LOG_WRN("unexpected id 0x%04X (expect 0x%04X)", data->flash_chip_id, W25Q128_CHIP_ID);
    }
    else {
        LOG_INF("w25q128 chip is Winbond, id = 0x%04X", data->flash_chip_id);
    }

    ret = w25q128_read_state(dev, &state);
    if (ret < 0) {
        LOG_ERR("ret state failed: %d", ret);
        return ret;
    }
    data->flash_state = state;

    if (data->flash_state != 0x00) {
        LOG_WRN("unexpected id 0x%02X (expect 0x%02X)", data->flash_state, 0x00);
    }
    else {
        LOG_INF("w25q128 state is okay, id = 0x%02X", data->flash_state);
    }
    /* 读写自检:向起始地址写入 0~255,读回逐个打印并校验 */
    uint8_t write_buf[W25Q128_PAGE_SIZE];
    uint8_t read_buf[W25Q128_PAGE_SIZE];
    int errors = 0;

    for (int i = 0; i < W25Q128_PAGE_SIZE; i++) {
        write_buf[i] = (uint8_t)i;
    }

    ret = w25q128_write(dev, 0x000000, write_buf, sizeof(write_buf));
    if (ret < 0) {
        LOG_ERR("w25q128 write test failed: %d", ret);
        return ret;
    }

    ret = w25q128_read(dev, 0x000000, read_buf, sizeof(read_buf));
    if (ret < 0) {
        LOG_ERR("w25q128 read test failed: %d", ret);
        return ret;
    }

    for (int i = 240; i < W25Q128_PAGE_SIZE; i++) {
        LOG_INF("addr 0x%06X = %3u (0x%02X)%s", i, read_buf[i], read_buf[i],
                (read_buf[i] == write_buf[i]) ? "" : "  <-- MISMATCH");
        if (read_buf[i] != write_buf[i]) {
            errors++;
        }
    }

    if (errors != 0) {
        LOG_ERR("w25q128 self test failed: %d bytes mismatch", errors);
        return -EIO;
    }
    LOG_INF("w25q128 self test passed: 256 bytes OK");

    return 0;
}

// Define the public API functions for the driver
static const struct w25q128_api w25q128_api_funcs = {
    .read = w25q128_read,
    .write = w25q128_write,
    .write_noe = w25q128_write_noe,
    .erase = w25q128_erase,
};

// Instance boilerplate  

#define W25Q128_SPI_INIT(inst)                                              \
    static const struct w25q128_spi_config w25q128_spi_cfg_##inst = {       \
        .spi = SPI_DT_SPEC_INST_GET(inst,                                   \
                SPI_WORD_SET(8) | SPI_OP_MODE_MASTER),                      \
    };                                                                      \
    static struct w25q128_spi_data w25q128_spi_data_##inst;                 \
                                                                            \
    DEVICE_DT_INST_DEFINE(inst,                                             \
                        w25q128_init,                                       \
                        NULL,                                               \
                        &w25q128_spi_data_##inst,                           \
                        &w25q128_spi_cfg_##inst,                            \
                        POST_KERNEL,                                        \
                        80,                                                 \
                        &w25q128_api_funcs);                                \
            
DT_INST_FOREACH_STATUS_OKAY(W25Q128_SPI_INIT)
