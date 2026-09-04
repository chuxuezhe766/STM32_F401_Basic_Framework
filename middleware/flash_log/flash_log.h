#ifndef __APP_FLASH_LOG_H__
#define __APP_FLASH_LOG_H__

#include <stdint.h>
#include <zephyr/device.h>

/* 单条日志正文最大长度(不含 2 字节记录头) */
#define FLASH_LOG_MAX_LEN 200u

/*
 * 简单追加式 flash 日志:
 * - 日志区固定在 LOG_BASE 起的 LOG_SIZE 字节(见 flash_log.c)
 * - 每条记录 = magic(1B) + len(1B) + 正文
 * - 上电 init 时扫描 0xFF 找到空闲位置,续写无需关心地址
 * - 写满后自动整区擦除,从头再来
 */
int flash_log_init(const struct device *dev);

/* 追加一条日志,text 超过 FLASH_LOG_MAX_LEN 时返回 -EINVAL */
int flash_log_append(const struct device *dev, const char *text);

/* 把日志区里的所有历史记录打印出来 */
int flash_log_dump(const struct device *dev);

#endif
