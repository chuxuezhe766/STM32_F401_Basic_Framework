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
#include "qmc5883l.h"
#include "sht30.h"
#include "attitude.h"
#include "flash_log.h"


#define DISPLAY_NODE DT_CHOSEN(zephyr_display)

// Settings
static const int32_t sleep_time_ms = 10;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* MPU6050 current full-scale ranges (configured in the driver via
 * GYRO_CONFIG/ACCEL_CONFIG): +/-2000 dps -> 16.4 LSB/(deg/s),
 * +/-16g -> 2048 LSB/g.
 * When swapping to another IMU, only these conversion constants change;
 * the attitude code is untouched. */
#define GYRO_LSB_PER_DPS     16.4f
#define ACCEL_LSB_PER_G      2048.0f
#define DEG_TO_RAD           0.01745329f

/* QMC5883L full scale +/-8Gs -> 3000 LSB/Gs, 1 Gs = 100 uT,
 * so uT = raw / 3000 * 100 = raw / 30 */
#define MAG_LSB_PER_GS       3000.0f
#define GS_TO_UT             100.0f
/* Hard-iron offset, measured with a full slow flat CW rotation and a
 * circle fit: the board's ferromagnetic parts shift the field by this
 * constant vector (in the QMC chip frame). Subtract before converting. */
#define MAG_BIAS_LSB         { 3519, 772, 484 }

/* Gyro auto-zero: average this many samples with the board at rest */
#define GYRO_CAL_SAMPLES     100
#define GYRO_CAL_INTERVAL_MS 5

int main(void)
{
    const struct device *display = DEVICE_DT_GET(DISPLAY_NODE);
    const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(user_ws2812_1));
    const struct device *flash = DEVICE_DT_GET(DT_NODELABEL(w25q128));
    const struct device *buzzer = DEVICE_DT_GET(DT_NODELABEL(buzzer));
    const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(mpu6050));
    const struct device *mag_dev = DEVICE_DT_GET(DT_NODELABEL(qmc5883l));
    const struct device *trh_dev = DEVICE_DT_GET(DT_NODELABEL(sht30));

    int ret;
    int state = 0;
    int loop_count = 0;
    struct led_rgb pixels[1];
    /* 9-axis fusion needs the magnetometer; if it is missing the app
     * falls back to 6-axis and yaw drifts again */
    bool have_mag = true;

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
        printk("flash init failed\r\n");
        return 0;
    }

    if (!device_is_ready(buzzer)) {
        printk("Buzzer device not ready\r\n");
        return 0;
    }

    if (!device_is_ready(gyro_dev)) {
        printk("gyro device not ready\r\n");
        return 0;
    }

    if (!device_is_ready(mag_dev)) {
        printk("mag device not ready, falling back to 6-axis\r\n");
        have_mag = false;
    }

    if (!device_is_ready(trh_dev)) {
        printk("trh device not ready\r\n");
        return 0;
    }    

    // Init the attitude filter: Mahony complementary filter, accel
    // magnitude outside 1g +/- 30% counts as an invalid sample
    struct attitude_cfg att_cfg = {
        .kp = 1.0f,
        .ki = 0.0f,
        .accel_tol_g = 0.3f,
        /* Mag heading on: axis mapping is identity and the hard-iron
         * offset is compensated in MAG_BIAS_LSB (calibrated by a full
         * flat rotation). Yaw now has an absolute reference. */
        .km = 1.0f,
    };
    ret = attitude_init(&att_cfg);
    if (ret < 0) {
        printk("attitude init failed: %d\r\n", ret);
        return 0;
    }

    // Gyro auto-zero to limit yaw drift: average 100 samples with the
    // board kept still, feed the mean to the filter as a fixed bias.
    // Without this, even a tiny gyro offset keeps integrating into yaw.
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    int16_t gyro_cal[3];
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (gyro_get_gyro(gyro_dev, gyro_cal) == 0) {
            for (int j = 0; j < 3; j++) {
                gyro_bias[j] += gyro_cal[j] / GYRO_LSB_PER_DPS
                                * DEG_TO_RAD / GYRO_CAL_SAMPLES;
            }
        }
        k_msleep(GYRO_CAL_INTERVAL_MS);
    }
    attitude_set_gyro_bias(gyro_bias);
    printk("gyro bias (rad/s): x=%.4f y=%.4f z=%.4f\r\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    // Init the flash log: scan the log area and continue from the
    // position of the last entry
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

    // Boot beep: 2kHz for 200ms (buzzer_tone is non-blocking)
    ret = buzzer_tone(buzzer, 2000, 200);
    if (ret < 0) {
        printk("buzzer tone failed: %d\r\n", ret);
    }

    // Configure the LED pin as output: without this the pin stays in
    // input mode and gpio_pin_set_dt only writes the output latch,
    // which has no effect on the LED
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
    if (ret < 0) {
        printk("LED configure failed: %d\r\n", ret);
        return 0;
    }

    // Set the GPIO as output
    ret = gpio_pin_set_dt(&led, state);
    if (ret < 0) {
        return 0;
    }

    /* Init CFB */
    if (cfb_framebuffer_init(display)) {
        printk("Framebuffer initialization failed\r\n");
        return 0;
    }

    // Init ws2812, off
    memset(pixels, 0, sizeof(pixels));
    /* Light pixel 0 in red */
    pixels[0].r = 25;
    pixels[0].g = 10;
    pixels[0].b = 0;
    ret = led_strip_update_rgb(strip, pixels, 1);
    printk("led_strip_update_rgb ret=%d\r\n", ret);

    uint8_t font_width;
    uint8_t font_height;
    cfb_get_font_size(display, 0, &font_width, &font_height);
    cfb_framebuffer_clear(display, true);

    // Time base for attitude updates, outside the loop so the first
    // dt is already correct
    int64_t attitude_last_ms = k_uptime_get();

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

        // Read the gyro 6-axis data, convert to physical units and
        // feed the attitude filter (9-axis when the mag is alive)
        int16_t acc[3], gyro[3], temp_raw;
        ret = gyro_get_all(gyro_dev, acc, gyro, &temp_raw);
        if (ret == 0) {
            float gyro_radps[3], accel_g[3];
            /* Board-frame mapping: the MPU6050 module is mounted upside
             * down on this board (raw accel z ~ -2048 at rest flat), so
             * rotate the chip frame 180 deg about X: negate y and z.
             * Accel and gyro live in the SAME chip frame -> same map. */
            accel_g[0] =  acc[0] / ACCEL_LSB_PER_G;
            accel_g[1] = -acc[1] / ACCEL_LSB_PER_G;
            accel_g[2] = -acc[2] / ACCEL_LSB_PER_G;
            gyro_radps[0] =  gyro[0] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
            gyro_radps[1] = -gyro[1] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
            gyro_radps[2] = -gyro[2] / GYRO_LSB_PER_DPS * DEG_TO_RAD;

            int16_t mag_raw[3] = {0, 0, 0};
            float mag_uT[3] = {0.0f, 0.0f, 0.0f};
            static const int16_t mag_bias[3] = MAG_BIAS_LSB;
            if (have_mag && mag_get_all(mag_dev, mag_raw) != 0) {
                have_mag = false;
                printk("mag read failed, falling back to 6-axis\r\n");
            }
            for (int i = 0; i < 3; i++) {
                mag_uT[i] = (mag_raw[i] - mag_bias[i])
                            / MAG_LSB_PER_GS * GS_TO_UT;
            }

            /* Debug: raw sensor frames every 10 loops, used to align
             * axis conventions and find the mag hard-iron offset */
            if ((loop_count % 10) == 0) {
                printk("raw acc: %d %d %d | mag: %d %d %d\r\n",
                       acc[0], acc[1], acc[2],
                       mag_raw[0], mag_raw[1], mag_raw[2]);
            }

            float dt = (float)k_uptime_delta(&attitude_last_ms) / 1000.0f;
            if (have_mag) {
                ret = attitude_update_mag(gyro_radps, accel_g, mag_uT, dt);
            } else {
                ret = attitude_update(gyro_radps, accel_g, dt);
            }
            if (ret == 0) {
                float roll, pitch, yaw;
                attitude_get_euler(&roll, &pitch, &yaw);
                printk("attitude: roll=%.2f pitch=%.2f yaw=%.2f deg\r\n",
                       roll, pitch, yaw);
                if (attitude_get_reject_count() != 0) {
                    printk("accel invalid samples: %u\r\n",
                           attitude_get_reject_count());
                }
                if (attitude_get_mag_reject_count() != 0) {
                    printk("mag invalid samples: %u\r\n",
                           attitude_get_mag_reject_count());
                }
            } else {
                printk("attitude update rejected: %d\r\n", ret);
            }

            printk("temp: %d.%02d C\r\n", temp_raw / 340 + 36,
                   (temp_raw % 340) * 100 / 340);
        } else {
            printk("gyro read failed: %d\r\n", ret);
        }

        // Append one log line to the W25Q128, continuing from the
        // last written address
        char log_line[FLASH_LOG_MAX_LEN + 1];
        snprintf(log_line, sizeof(log_line), "loop %d, LED state: %d",
                 loop_count, state);
        ret = flash_log_append(flash, log_line);
        if (ret < 0) {
            printk("flash log append failed: %d\r\n", ret);
        }

        // Every 10 loops, dump the whole log history from flash
        if ((loop_count % 10) == 9) {
            flash_log_dump(flash);
        }
        loop_count++;

        float temp, hum_pct;
        ret = th_get_all(trh_dev, &temp, &hum_pct);
        if (ret < 0) {
            printk("trh read failed: %d\r\n", ret);
        } else {
            printk("trh temp is %.2f, hum_pct is %.2f\r\n", temp, hum_pct);
        }

        // Sleep
        k_msleep(sleep_time_ms);
    }

    return 0;
}
