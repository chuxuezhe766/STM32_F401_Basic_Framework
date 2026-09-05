#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>
#include <zephyr/kernel.h>

/*
 * 6/9-axis attitude estimation (Mahony complementary filter), middleware.
 *
 * Decoupled from any specific sensor: this module only takes physical
 * units -- gyro rate in rad/s, accel in g, magnetic field in uT.
 * To swap the IMU (e.g. to a BMI088) or the compass (e.g. to an
 * HMC5883L), just convert the new chips' raw readings to those units
 * in the app layer. No change needed in this module.
 *
 * Two update paths:
 *  - attitude_update():      6-axis, gyro + accel. Roll/pitch are
 *    corrected by the accelerometer, yaw has no absolute reference
 *    and drifts slowly.
 *  - attitude_update_mag():  9-axis, gyro + accel + mag. The magnetic
 *    heading corrects yaw, so it no longer drifts (a constant
 *    offset may remain until hard-iron calibration is done).
 *
 * Error detection:
 *  - Every API checks pointers, init state, and value sanity
 *    (NaN/Inf, out-of-range dt)
 *  - If accel magnitude is outside 1g +/- tolerance (vibration or
 *    maneuvering), the accel correction is skipped for that sample
 *    and counted; watch attitude_get_reject_count()
 *  - Mag samples outside the Earth-field range (~25-65 uT) are
 *    skipped and counted; watch attitude_get_mag_reject_count()
 *  - If the quaternion gets corrupted, it is reset to identity
 *
 * Drift handling:
 *  - The quaternion is aligned with the first valid accel sample, so
 *    there is no convergence transient at boot
 *  - Gyro bias: startup bias can be set via attitude_set_gyro_bias();
 *    additionally, while every axis stays below ~1.7 dps the module
 *    re-learns the bias automatically, which tracks the zero-rate
 *    drift that occurs as the chip warms up after power-on
 */

struct attitude_cfg {
    /* Proportional gain: how hard the accel vector corrects the
     * attitude estimate. 0 = gyro integration only. Typical 0.5 ~ 2.0 */
    float kp;
    /* Integral gain: estimates and compensates gyro bias online.
     * Typical 0 ~ 0.1, 0 = disabled */
    float ki;
    /* Accel validity tolerance: samples with |a| deviating from 1g by
     * more than this ratio are rejected for correction.
     * e.g. 0.3 means |a| outside 0.7g ~ 1.3g is invalid. Range (0, 1) */
    float accel_tol_g;
    /* Magnetometer heading gain, used by attitude_update_mag() only:
     * how hard the magnetic heading corrects yaw. 0 = ignore the
     * magnetometer. Typical 0.5 ~ 2.0 */
    float km;
};

/*
 * Initialize the filter, attitude reset to level (identity quaternion).
 * Returns -EINVAL if cfg is NULL, contains NaN, or values are out of range.
 */
int attitude_init(const struct attitude_cfg *cfg);

/*
 * 6-axis update: feed one gyro + accel sample and advance the filter.
 *
 * gyro_radps: 3-axis angular rate, rad/s
 * accel_g   : 3-axis acceleration, g
 * dt_s      : time since last update in seconds, must be in (0, 0.5]
 *
 * Returns 0 on success. Returns -EINVAL on NULL pointers, NaN/Inf
 * values, or dt out of range. Returns -EACCES if attitude_init()
 * was not called. Invalid accel is not a failure: that update just
 * skips accel correction and bumps the reject counter.
 */
int attitude_update(const float gyro_radps[3], const float accel_g[3],
                    float dt_s);

/*
 * 9-axis update: same as attitude_update() plus a magnetometer sample
 * for drift-free yaw.
 *
 * mag_uT: 3-axis magnetic field in microtesla. Only the direction is
 * used, but the magnitude is sanity-checked against the Earth field
 * (~25-65 uT); samples outside [10, 150] uT are skipped and counted.
 * QMC5883L at +/-8Gs: uT = raw / 3000 * 100 = raw / 30.
 *
 * Return value and error codes are the same as attitude_update().
 * An invalid mag sample is not a failure: that update just skips the
 * heading correction.
 */
int attitude_update_mag(const float gyro_radps[3], const float accel_g[3],
                        const float mag_uT[3], float dt_s);

/*
 * Set a fixed gyro bias (rad/s) subtracted from every sample, e.g.
 * measured by averaging gyro readings at rest during startup.
 * Without this, any residual gyro bias integrates into yaw drift.
 */
int attitude_set_gyro_bias(const float bias_radps[3]);

/* Read Euler angles in degrees. Roll: X axis, pitch: Y axis, yaw:
 * Z axis (Z-Y-X order). */
int attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

/* Read the current quaternion q[4] = {w, x, y, z}. */
int attitude_get_quat(float q[4]);

/* Number of updates that skipped accel correction since boot. */
uint32_t attitude_get_reject_count(void);

/* Number of updates that skipped mag heading correction since boot. */
uint32_t attitude_get_mag_reject_count(void);

#endif
