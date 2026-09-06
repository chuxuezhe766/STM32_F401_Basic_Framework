#ifndef __ZEPHYR_DRIVERS_KT6368A_H__
#define __ZEPHYR_DRIVERS_KT6368A_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

/*
 * Generic UART bluetooth transparent-bridge interface.
 *
 * Public functions use the bt_ prefix (same idea as gyro_ in
 * mpu6050.h, mag_ in qmc5883l.h, th_ in sht30.h, baro_ in bmp280.h
 * and flash_ in w25q128.h) and never expose the chip model: swapping
 * KT6368A for another UART transparent bluetooth chip (JDY-31,
 * HC-05, ...) only requires the new driver to implement the same
 * api struct. Neither the app nor any other code needs any change.
 *
 * The chip is a serial bridge: before a phone/peer connects, serial
 * traffic is AT commands (format "AT+<CMD>\r\n", pure ASCII); after
 * the connection is up the same serial line becomes a raw transparent
 * pipe and AT commands are no longer recognized.
 */

// Define KT6368A API
struct kt6368a_api {
    /* Push bytes over the bluetooth link (transparent mode). Sends
     * in one contiguous burst: the chip frames serial input with a
     * ~10ms RX timeout interrupt, so gaps inside one message make
     * it arrive split at the peer. */
    int (*send)(const struct device *dev, const uint8_t *data,
                size_t len);
    /* Poll for bytes received from the bluetooth link. Returns the
     * number of bytes copied (stops early after an idle gap on the
     * line), 0 when nothing arrived within timeout_ms. */
    int (*recv)(const struct device *dev, uint8_t *buf, size_t len,
                uint32_t timeout_ms);
    /* Send one raw AT command (without CR/LF, appended here) and
     * collect the reply into resp as a NUL-terminated string.
     * Only works while no peer is connected. */
    int (*at_command)(const struct device *dev, const char *cmd,
                      char *resp, size_t resp_max);
    /* Communication check (AT+QT): 0 when the chip answers */
    int (*ping)(const struct device *dev);
    /* Read the bluetooth name / MAC address / firmware version as a
     * string (raw chip reply) */
    int (*get_name)(const struct device *dev, char *buf, size_t len);
    int (*get_mac)(const struct device *dev, char *buf, size_t len);
    int (*get_version)(const struct device *dev, char *buf, size_t len);
    /* Soft reset (AT+CZ), reply is not waited for */
    int (*reset)(const struct device *dev);
    /* Read the STA status pin: true = a bluetooth peer is connected.
     * Only available when the devicetree node has sta-gpios, else
     * returns -ENOSYS. */
    int (*link_status)(const struct device *dev, bool *connected);
};

static inline int bt_send(const struct device *dev, const uint8_t *data,
                          size_t len)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->send == NULL) {
        return -ENOSYS;
    }
    return api->send(dev, data, len);
}

static inline int bt_recv(const struct device *dev, uint8_t *buf,
                          size_t len, uint32_t timeout_ms)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->recv == NULL) {
        return -ENOSYS;
    }
    return api->recv(dev, buf, len, timeout_ms);
}

static inline int bt_at(const struct device *dev, const char *cmd,
                        char *resp, size_t resp_max)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->at_command == NULL) {
        return -ENOSYS;
    }
    return api->at_command(dev, cmd, resp, resp_max);
}

static inline int bt_ping(const struct device *dev)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->ping == NULL) {
        return -ENOSYS;
    }
    return api->ping(dev);
}

static inline int bt_get_name(const struct device *dev, char *buf,
                              size_t len)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->get_name == NULL) {
        return -ENOSYS;
    }
    return api->get_name(dev, buf, len);
}

static inline int bt_get_mac(const struct device *dev, char *buf,
                             size_t len)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->get_mac == NULL) {
        return -ENOSYS;
    }
    return api->get_mac(dev, buf, len);
}

static inline int bt_get_version(const struct device *dev, char *buf,
                                 size_t len)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->get_version == NULL) {
        return -ENOSYS;
    }
    return api->get_version(dev, buf, len);
}

static inline int bt_reset(const struct device *dev)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->reset == NULL) {
        return -ENOSYS;
    }
    return api->reset(dev);
}

static inline int bt_connected(const struct device *dev, bool *connected)
{
    const struct kt6368a_api *api = dev->api;
    if (api == NULL || api->link_status == NULL) {
        return -ENOSYS;
    }
    return api->link_status(dev, connected);
}

#endif
