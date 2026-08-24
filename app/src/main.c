#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include "ws2812.h"

#define DISPLAY_NODE DT_CHOSEN(zephyr_display)

// Settings
static const int32_t sleep_time_ms = 1000;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    const struct device *display = DEVICE_DT_GET(DISPLAY_NODE);
    const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(user_ws2812_1));

    int ret;
    int state = 0;
    struct led_rgb pixels[1];

    // Make sure that the GPIO was initialized
    if (!gpio_is_ready_dt(&led)) {
        printk("GPIO not ready\r\n");
        return 0;
    }

    // Make sure that the Device was initialized
    if (!device_is_ready(display)) {
        printk("Display device not ready\r\n");
        return 0;
    }

    if (!device_is_ready(strip)) {
        printk("LED Strip device not ready\r\n");
        return 0;
    }

    // Set the GPIO as output
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
    if (ret < 0) {
        return 0;
    }

    /* 初始化 CFB*/
    if (cfb_framebuffer_init(display)) {
        printk("Framebuffer initialization failed\r\n");
        return 0;
    }

    // 初始化ws2812,熄灭
    memset(pixels, 0, sizeof(pixels));
    /* 点亮第 0 颗为红色 */
    pixels[0].r = 0;
    pixels[0].g = 22;
    pixels[0].b = 0;

    uint8_t font_width;
    uint8_t font_height;
    cfb_get_font_size(display, 0, &font_width, &font_height);
    cfb_framebuffer_clear(display, true);
    // Do forever  
    while (1) {
        // Change the state of the pin and print
        state = !state;
        printk("LED state: %d\r\n", state);

        cfb_framebuffer_clear(display, false);
        cfb_print(display, "Hello Zephyr", 0, 0);
        cfb_framebuffer_finalize(display);

        // Set pin state
        ret = gpio_pin_set_dt(&led, state);
        if (ret < 0) {
            return 0;
        }
    ret = led_strip_update_rgb(strip, pixels, 1);
    printk("led_strip_update_rgb ret=%d\r\n", ret);
        // Sleep
        k_msleep(sleep_time_ms);
    }

    return 0;
}
