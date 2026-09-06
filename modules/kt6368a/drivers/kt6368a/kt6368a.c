#define DT_DRV_COMPAT qinyue_kt6368a

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>
#include "kt6368a.h"

LOG_MODULE_REGISTER(kt6368a, LOG_LEVEL_INF);

/* The AT commands of kt6368a (pure ASCII, terminated with \r\n,
 * only recognized while no bluetooth peer is connected) */
#define KT6368A_CMD_PING        "AT+QT"    /* communication check, reply
                                              starts with "QT" */
#define KT6368A_CMD_VERSION     "AT+TD"    /* firmware version */
#define KT6368A_CMD_MAC         "AT+TN"    /* MAC address */
#define KT6368A_CMD_NAME        "AT+TM"    /* bluetooth name */
#define KT6368A_CMD_RESET       "AT+CZ"    /* soft reset */

/* After the chip has answered, this much silence on the line means
 * the reply is complete (the chip itself frames with a ~10ms idle
 * interrupt, so a longer gap safely covers one full reply) */
#define KT6368A_REPLY_GAP_MS    30
/* Overall cap for one AT exchange */
#define KT6368A_AT_TIMEOUT_MS   200
/* The chip needs this long to come back after a soft reset */
#define KT6368A_RESET_TIME_MS   100
/* IRQ-fed receive ring size: plenty for one transparent burst */
#define KT6368A_RX_BUF_SIZE     256

// Configure
struct kt6368a_config {
    /* Pointer to the UART device driver (the DT bus node).
     * struct uart_dt_spec does not exist in Zephyr, so we hold the
     * device pointer directly instead. */
    const struct device *uart;
    /* STA status pin, optional: high = peer connected. .port is NULL
     * when the devicetree node has no sta-gpios property. */
    struct gpio_dt_spec sta;
};

// Data
struct kt6368a_data {
    struct k_mutex lock;
    /* Receive ring fed by the UART ISR: bytes are captured the moment
     * they arrive and recv()/collect() drain it at their own pace.
     * Replaces the old polled design whose 1-byte UART buffer lost
     * whole bursts during the multi-millisecond loop windows. */
    struct ring_buf rx_ring;
    uint8_t rx_mem[KT6368A_RX_BUF_SIZE];
};

static void kt6368a_write_bytes(const struct kt6368a_config *cfg,
                                const uint8_t *data, size_t len)
{
    /* poll_out per byte keeps the whole message contiguous on the
     * line: the chip's ~10ms RX timeout interrupt frames input, so
     * gaps inside one command make it get treated as data */
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(cfg->uart, data[i]);
    }
}

static void kt6368a_write_str(const struct kt6368a_config *cfg,
                              const char *s)
{
    kt6368a_write_bytes(cfg, (const uint8_t *)s, strlen(s));
}

/* UART ISR: push every received byte into the ring the moment it
 * arrives, so nothing depends on when the thread happens to poll */
static void kt6368a_uart_isr(const struct device *uart, void *user_data)
{
    const struct device *dev = user_data;
    struct kt6368a_data *data = dev->data;

    uart_irq_update(uart);
    while (uart_irq_rx_ready(uart)) {
        uint8_t tmp[16];
        int n = uart_fifo_read(uart, tmp, sizeof(tmp));

        if (n <= 0) {
            break;
        }
        ring_buf_put(&data->rx_ring, tmp, (uint32_t)n);
    }
}

/* Drop any stale bytes accumulated in the rx ring */
static void kt6368a_drain_rx(struct kt6368a_data *data)
{
    ring_buf_reset(&data->rx_ring);
}

/* Collect bytes from the rx ring until an idle gap (reply complete)
 * or the overall timeout. Returns the number of bytes copied. */
static size_t kt6368a_collect(struct kt6368a_data *data,
                              uint8_t *buf, size_t max,
                              uint32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;
    int64_t gap_deadline = 0;
    size_t n = 0;

    /* The ring is filled by the ISR, so the thread can sleep and
     * still lose nothing */
    while (n < max && k_uptime_get() < deadline) {
        uint8_t ch;
        uint32_t got = ring_buf_get(&data->rx_ring, &ch, 1);

        if (got > 0) {
            buf[n++] = ch;
            gap_deadline = k_uptime_get() + KT6368A_REPLY_GAP_MS;
        } else if (n > 0 && k_uptime_get() >= gap_deadline) {
            /* silence for one full reply gap: frame complete */
            break;
        } else {
            k_sleep(K_MSEC(1));
        }
    }
    return n;
}

static int kt6368a_send(const struct device *dev, const uint8_t *data,
                        size_t len)
{
    const struct kt6368a_config *cfg = dev->config;
    struct kt6368a_data *data_priv = dev->data;

    if (data == NULL || len == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&data_priv->lock, K_FOREVER);
    kt6368a_write_bytes(cfg, data, len);
    k_mutex_unlock(&data_priv->lock);
    return 0;
}

static int kt6368a_recv(const struct device *dev, uint8_t *buf,
                        size_t len, uint32_t timeout_ms)
{
    struct kt6368a_data *data_priv = dev->data;
    size_t n;

    if (buf == NULL || len == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&data_priv->lock, K_FOREVER);
    n = kt6368a_collect(data_priv, buf, len, timeout_ms);
    k_mutex_unlock(&data_priv->lock);
    return (int)n;
}

/* Send "AT+<cmd>\r\n" in one piece, collect the reply and return it
 * as a NUL-terminated string. */
static int kt6368a_at_command(const struct device *dev, const char *cmd,
                              char *resp, size_t resp_max)
{
    const struct kt6368a_config *cfg = dev->config;
    struct kt6368a_data *data_priv = dev->data;
    size_t n;

    if (cmd == NULL || resp == NULL || resp_max < 2) {
        return -EINVAL;
    }

    k_mutex_lock(&data_priv->lock, K_FOREVER);
    kt6368a_drain_rx(data_priv);
    kt6368a_write_str(cfg, cmd);
    kt6368a_write_str(cfg, "\r\n");
    n = kt6368a_collect(data_priv, (uint8_t *)resp, resp_max - 1,
                        KT6368A_AT_TIMEOUT_MS);
    k_mutex_unlock(&data_priv->lock);

    if (n == 0) {
        LOG_ERR("no reply to %s (connected? wiring? baud?)", cmd);
        return -ETIMEDOUT;
    }
    resp[n] = '\0';
    /* Strip the trailing CR/LF so callers can use the string as is */
    while (n > 0 && (resp[n - 1] == '\r' || resp[n - 1] == '\n')) {
        resp[--n] = '\0';
    }
    return (int)n;
}

static int kt6368a_ping(const struct device *dev)
{
    char resp[16];
    int ret = kt6368a_at_command(dev, KT6368A_CMD_PING,
                                 resp, sizeof(resp));

    if (ret < 0) {
        return ret;
    }
    /* Datasheet: the communication check reply starts with "QT" */
    if (strncmp(resp, "QT", 2) != 0) {
        LOG_WRN("unexpected ping reply: %s", resp);
        return -EIO;
    }
    return 0;
}

static int kt6368a_get_name(const struct device *dev, char *buf,
                            size_t len)
{
    return kt6368a_at_command(dev, KT6368A_CMD_NAME, buf, len);
}

static int kt6368a_get_mac(const struct device *dev, char *buf,
                           size_t len)
{
    return kt6368a_at_command(dev, KT6368A_CMD_MAC, buf, len);
}

static int kt6368a_get_version(const struct device *dev, char *buf,
                               size_t len)
{
    return kt6368a_at_command(dev, KT6368A_CMD_VERSION, buf, len);
}

static int kt6368a_reset(const struct device *dev)
{
    const struct kt6368a_config *cfg = dev->config;
    struct kt6368a_data *data_priv = dev->data;

    k_mutex_lock(&data_priv->lock, K_FOREVER);
    kt6368a_drain_rx(data_priv);
    kt6368a_write_str(cfg, KT6368A_CMD_RESET);
    kt6368a_write_str(cfg, "\r\n");
    k_mutex_unlock(&data_priv->lock);

    /* The chip reboots and may not answer; just give it time */
    k_msleep(KT6368A_RESET_TIME_MS);
    return 0;
}

static int kt6368a_link_status(const struct device *dev, bool *connected)
{
    const struct kt6368a_config *cfg = dev->config;

    if (cfg->sta.port == NULL) {
        return -ENOSYS;
    }
    *connected = gpio_pin_get_dt(&cfg->sta) > 0;
    return 0;
}

// Initialize kt6368a
static int kt6368a_init(const struct device *dev)
{
    const struct kt6368a_config *cfg = dev->config;
    struct kt6368a_data *data = dev->data;
    int ret;

    // Check uart
    if (!device_is_ready(cfg->uart)) {
        LOG_ERR("UART not ready");
        return -ENODEV;
    }

    // RX: interrupt + ring buffer (see struct kt6368a_data)
    ring_buf_init(&data->rx_ring, sizeof(data->rx_mem), data->rx_mem);
    ret = uart_irq_callback_user_data_set(cfg->uart, kt6368a_uart_isr,
                                          (void *)dev);
    if (ret) {
        LOG_ERR("uart irq callback set failed: %d", ret);
        return ret;
    }
    uart_irq_rx_enable(cfg->uart);

    // STA pin as input when provided in the devicetree
    if (cfg->sta.port != NULL) {
        int ret = gpio_pin_configure_dt(&cfg->sta, GPIO_INPUT);

        if (ret) {
            LOG_ERR("STA pin configure failed: %d", ret);
            return ret;
        }
    }

    // Init mutex
    k_mutex_init(&data->lock);

    /* Communication check: warn only, the module may already be in a
     * connected (transparent) state where AT commands are not seen */
    if (kt6368a_ping(dev) != 0) {
        LOG_WRN("no AT reply, module connected or not wired yet");
    } else {
        LOG_INF("KT6368A detected");
    }

    LOG_INF("KT6368A init done");
    return 0;
}

// Define the public API functions for the driver
static const struct kt6368a_api kt6368a_api_funcs = {
    .send = kt6368a_send,
    .recv = kt6368a_recv,
    .at_command = kt6368a_at_command,
    .ping = kt6368a_ping,
    .get_name = kt6368a_get_name,
    .get_mac = kt6368a_get_mac,
    .get_version = kt6368a_get_version,
    .reset = kt6368a_reset,
    .link_status = kt6368a_link_status,
};

// Instance boilerplate

#define KT6368A_STA_SPEC(inst) COND_CODE_1(                             \
    DT_INST_NODE_HAS_PROP(inst, sta_gpios),                             \
    (GPIO_DT_SPEC_INST_GET(inst, sta_gpios)),                           \
    ({ .port = NULL }))

#define KT6368A_UART_INIT(inst)                                         \
    static const struct kt6368a_config kt6368a_cfg_##inst = {           \
        .uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                       \
        .sta = KT6368A_STA_SPEC(inst),                                  \
    };                                                                  \
    static struct kt6368a_data kt6368a_data_##inst;                     \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                        kt6368a_init,                                   \
                        NULL,                                           \
                        &kt6368a_data_##inst,                           \
                        &kt6368a_cfg_##inst,                            \
                        POST_KERNEL,                                    \
                        80,                                             \
                        &kt6368a_api_funcs);                            \

DT_INST_FOREACH_STATUS_OKAY(KT6368A_UART_INIT)
