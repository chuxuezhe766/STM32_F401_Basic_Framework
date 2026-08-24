#ifndef ZEPHYR_DRIVERS_WS2812_H_
#define ZEPHYR_DRIVERS_WS2812_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/sys/util.h>

static inline int ws2812_set_color(const struct device *dev,
                                   uint32_t index,
                                   uint8_t r,
                                   uint8_t g,
                                   uint8_t b)
{
    struct led_rgb pixels = {
        .r = r,
        .g = g,
        .b = b,
    };

    return led_strip_update_rgb(dev, &pixels, 1);
}

#endif