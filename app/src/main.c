#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/buzzer.h>
#include <zephyr/display/cfb.h>
#include "ws2812.h"
#include "w25q128.h"
#include "mpu6050.h"
#include "flash_log.h"


#define DISPLAY_NODE DT_CHOSEN(zephyr_display)

// Settings
static const int32_t sleep_time_ms = 1000;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    const struct device *display = DEVICE_DT_GET(DISPLAY_NODE);
    const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(user_ws2812_1));
    const struct device *flash = DEVICE_DT_GET(DT_NODELABEL(w25q128));
    const struct device *buzzer = DEVICE_DT_GET(DT_NODELABEL(buzzer));
    const struct device *mpu6050 = DEVICE_DT_GET(DT_NODELABEL(mpu6050));

    int ret;
    int state = 0;
    int loop_count = 0;
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

    if (!device_is_ready(flash)) {
      printk("W25Q128 init failed\r\n");
      return 0;
    }

    if (!device_is_ready(buzzer)) {
      printk("Buzzer device not ready\r\n");
      return 0;
    }

    if (!device_is_ready(mpu6050)) {
      printk("Mpu6050 device not ready\r\n");
      return 0;
    }

    // 初始化 flash 日志:扫描日志区,接着上次的位置继续写
    ret = flash_log_init(flash);
    if (ret < 0) {
        printk("flash log init failed: %d\r\n", ret);
        return 0;
    }

    ret = flash_log_append(flash, "system boot");
    if (ret < 0) {
        printk("flash log append failed: %d\r\n", ret);
        return 0;
    }

    // 开机响一声:2kHz,响 200ms(buzzer_tone 非阻塞,驱动到点自动停)
    ret = buzzer_tone(buzzer, 2000, 200);
    if (ret < 0) {
        printk("buzzer tone failed: %d\r\n", ret);
    }

    // Set the GPIO as output
    ret = gpio_pin_set_dt(&led, state);
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
    pixels[0].r = 25;
    pixels[0].g = 10;
    pixels[0].b = 0;
    ret = led_strip_update_rgb(strip, pixels, 1);
    printk("led_strip_update_rgb ret=%d\r\n", ret);

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

        // 读取MPU6050六轴数据,打印加速度/温度/陀螺仪
        int16_t acc[3], gyro[3], temp_raw;
        ret = mpu6050_get_all(mpu6050, acc, gyro, &temp_raw);
        if (ret == 0) {
            printk("acc raw:  X=%d Y=%d Z=%d\r\n", acc[0], acc[1], acc[2]);
            printk("gyro raw: X=%d Y=%d Z=%d\r\n", gyro[0], gyro[1], gyro[2]);
            printk("temp: %d.%02d C\r\n", temp_raw / 340 + 36,
                   (temp_raw % 340) * 100 / 340);
        } else {
            printk("mpu6050 read failed: %d\r\n", ret);
        }

        // 记一条日志到 W25Q128,接着上次的地址续写
        char log_line[FLASH_LOG_MAX_LEN + 1];
        snprintf(log_line, sizeof(log_line), "loop %d, LED state: %d",
                 loop_count, state);
        ret = flash_log_append(flash, log_line);
        if (ret < 0) {
            printk("flash log append failed: %d\r\n", ret);
        }

        // 每 10 圈把日志区里的历史记录全部打印一遍
        if ((loop_count % 10) == 9) {
            flash_log_dump(flash);
        }
        loop_count++;

        // Sleep
        k_msleep(sleep_time_ms);
    }

    return 0;
}
