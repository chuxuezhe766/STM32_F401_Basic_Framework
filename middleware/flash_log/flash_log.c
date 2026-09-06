#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "w25q128.h"
#include "flash_log.h"

LOG_MODULE_REGISTER(flash_log, LOG_LEVEL_INF);

/* 日志区放在 1MB 偏移处,占 64KB(16 个 4KB 扇区),
 * 避开驱动自检占用的 0 地址附近 */
#define LOG_BASE        0x00100000u
#define LOG_SIZE        0x00010000u
#define LOG_END         (LOG_BASE + LOG_SIZE)

/* 每条记录: magic(1B) + len(1B) + 正文(len 字节) */
#define LOG_HDR_SIZE    2u
#define LOG_MAGIC       0xA5u

/* 下一条日志的写入地址和当前条数,由 init 扫描得出 */
static uint32_t next_addr;
static uint32_t record_count;

/* 整区擦除,从头开始记录 */
static int flash_log_format(const struct device *dev)
{
    int ret = flash_erase(dev, LOG_BASE, LOG_SIZE);

    if (ret) {
        return ret;
    }
    next_addr = LOG_BASE;
    record_count = 0;
    LOG_INF("log region erased, restart from 0x%06X", LOG_BASE);
    return 0;
}

int flash_log_init(const struct device *dev)
{
    uint8_t hdr[LOG_HDR_SIZE];
    int ret;

    next_addr = LOG_BASE;
    record_count = 0;

    /* 逐条向后走:合法记录(magic + 合理长度)就跳过,
     * 一旦读到 0xFF 或坏数据,就是空闲区的开头 */
    while (next_addr + LOG_HDR_SIZE <= LOG_END) {
        ret = flash_read(dev, next_addr, hdr, sizeof(hdr));
        if (ret) {
            return ret;
        }
        if (hdr[0] != LOG_MAGIC ||
            hdr[1] == 0 ||
            next_addr + LOG_HDR_SIZE + hdr[1] > LOG_END) {
            break;
        }
        next_addr += LOG_HDR_SIZE + hdr[1];
        record_count++;
    }

    LOG_INF("flash log: %u records, next write addr 0x%06X",
            record_count, next_addr);
    return 0;
}

int flash_log_append(const struct device *dev, const char *text)
{
    uint8_t buf[LOG_HDR_SIZE + FLASH_LOG_MAX_LEN];
    size_t len;
    int ret;

    if (text == NULL) {
        return -EINVAL;
    }
    len = strlen(text);
    if (len == 0 || len > FLASH_LOG_MAX_LEN) {
        return -EINVAL;
    }

    /* 剩余空间放不下这一条,就整区擦除重来 */
    if (next_addr + LOG_HDR_SIZE + len > LOG_END) {
        ret = flash_log_format(dev);
        if (ret) {
            return ret;
        }
    }

    buf[0] = LOG_MAGIC;
    buf[1] = (uint8_t)len;
    memcpy(&buf[LOG_HDR_SIZE], text, len);

    /* 区域已确认是擦除态,直接编程即可 */
    ret = flash_write_noe(dev, next_addr, buf, LOG_HDR_SIZE + len);
    if (ret) {
        return ret;
    }
    next_addr += LOG_HDR_SIZE + len;
    record_count++;
    return 0;
}

/* 转储一次最多打印的记录数(最新的几条),
 * 记录多了全量打印会淹没日志缓冲 */
#define DUMP_MAX_RECORDS 5u

int flash_log_dump(const struct device *dev)
{
    uint8_t payload[FLASH_LOG_MAX_LEN + 1];
    uint8_t hdr[LOG_HDR_SIZE];
    uint32_t addr = LOG_BASE;
    uint32_t idx = 0;
    int ret;

    printk("---- flash log dump ----\r\n");
    while (addr + LOG_HDR_SIZE <= LOG_END) {
        ret = flash_read(dev, addr, hdr, sizeof(hdr));
        if (ret) {
            return ret;
        }
        if (hdr[0] != LOG_MAGIC || hdr[1] == 0 ||
            addr + LOG_HDR_SIZE + hdr[1] > LOG_END) {
            break;      /* 到达空闲区或记录损坏 */
        }
        ret = flash_read(dev, addr + LOG_HDR_SIZE, payload, hdr[1]);
        if (ret) {
            return ret;
        }
        payload[hdr[1]] = '\0';
        /* 只打印最新的 DUMP_MAX_RECORDS 条,总数照常统计 */
        if (idx + DUMP_MAX_RECORDS >= record_count) {
            printk("[%3u] %s\r\n", idx, payload);
        }
        addr += LOG_HDR_SIZE + hdr[1];
        idx++;
    }
    printk("---- end of dump: %u records (last %u shown) ----\r\n",
           idx, idx < DUMP_MAX_RECORDS ? idx : DUMP_MAX_RECORDS);
    return 0;
}
