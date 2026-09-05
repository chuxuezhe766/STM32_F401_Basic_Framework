#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <zephyr/logging/log.h>
#include "attitude.h"

LOG_MODULE_REGISTER(attitude, LOG_LEVEL_INF);

#define ATTITUDE_MAX_DT_S   0.5f        /* dt above this is treated as an error */
#define ATTITUDE_RAD_TO_DEG 57.29577951f
#define ATTITUDE_PI         3.14159265f

/* Accel correction is skipped while the board is "quiet" below this
 * raw rate on every axis (~1.7 dps)... */
#define ATTITUDE_QUIET_RADPS  0.03f
/* ...and only re-learned after it has stayed quiet this long */
#define ATTITUDE_QUIET_TIME_S 1.0f

/* Mag samples outside this range are not the Earth field (~25-65 uT):
 * saturated, distorted by nearby metal, or dead. Correction skipped. */
#define ATTITUDE_MAG_MIN_UT 10.0f
#define ATTITUDE_MAG_MAX_UT 150.0f

/* ---- Module-private state, only touched through the public API ---- */

/* Attitude quaternion q = {w, x, y, z} */
static float q0 = 1.0f, q1, q2, q3;
/* Set once the quaternion has been aligned with the first valid accel */
static bool seeded;
/* Integral of the accel error, used to estimate gyro bias online */
static float integ_x, integ_y, integ_z;
/* Fixed gyro bias (rad/s) subtracted from every sample */
static float gyro_bias[3];
/* Accumulator for stationary bias re-learning */
static float quiet_sum[3];
static float quiet_time;
static struct attitude_cfg cfg;
static float two_kp, two_ki, two_km;
static bool inited;
static uint32_t reject_count;
static uint32_t mag_reject_count;

/* ---- Private functions ---- */

/* Value sanity: reject NaN (v != v) and Inf (beyond finite float range) */
static bool float_ok(float v)
{
    return (v == v) && (v > -3.4028235e38f) && (v < 3.4028235e38f);
}

static bool vec3_ok(const float v[3])
{
    return float_ok(v[0]) && float_ok(v[1]) && float_ok(v[2]);
}

/* Wrap an angle in radians to [-pi, pi] */
static float wrap_pi(float a)
{
    if (a > ATTITUDE_PI) {
        a -= 2.0f * ATTITUDE_PI;
    } else if (a < -ATTITUDE_PI) {
        a += 2.0f * ATTITUDE_PI;
    }
    return a;
}

/* Normalize the quaternion; if its norm is broken (~0 or NaN), reset to
 * identity so one bad sample cannot poison everything after it */
static void quat_normalize(void)
{
    float n = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;

    if (!float_ok(n) || n < 1e-6f) {
        LOG_ERR("quaternion corrupted, reset to identity");
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        integ_x = integ_y = integ_z = 0.0f;
        return;
    }
    float recip = 1.0f / sqrtf(n);
    q0 *= recip;
    q1 *= recip;
    q2 *= recip;
    q3 *= recip;
}

/*
 * Align the quaternion with the first valid accel sample: roll/pitch
 * from the gravity vector, yaw starts at 0 (the magnetometer pulls it
 * to the real heading within a few updates once km > 0). This avoids
 * the large Euler-angle transient the filter would otherwise show
 * while converging from identity.
 */
static void seed_from_accel(float ax, float ay, float az)
{
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;
    integ_x = integ_y = integ_z = 0.0f;
    seeded = true;
    LOG_INF("attitude seeded from accel: roll=%.1f pitch=%.1f deg",
            roll * ATTITUDE_RAD_TO_DEG, pitch * ATTITUDE_RAD_TO_DEG);
}

/* Euler roll/pitch of the current quaternion, in radians */
static void quat_get_roll_pitch(float *roll, float *pitch)
{
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2));

    /* sin(pitch): rounding may push the asinf argument slightly out of
     * [-1, 1], so clamp it */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp);
}

/*
 * One Mahony filter step:
 * 1. Cross product of the gravity direction estimated from the
 *    quaternion and the one measured by the accel gives the attitude
 *    error
 * 2. In 9-axis mode (mag != NULL): de-tilt the mag vector with the
 *    current roll/pitch (kept level by the accel); the horizontal
 *    components give the magnetic heading. The heading error feeds
 *    into gz, giving yaw an absolute reference - no more drift.
 * 3. Errors are fed back into the gyro rate with gain kp (plus ki
 *    integral), acting like a virtual gyro-bias correction
 * 4. First-order integration q_dot = 0.5 * q (x) w, then normalize
 * When |accel| is outside 1g +/- tolerance (vibration/maneuvering),
 * step 1 is skipped and only the gyro (and mag heading) is used.
 */
static void mahony_step(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt,
                        const float mag_uT[3])
{
    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    if (accel_norm > (1.0f - cfg.accel_tol_g) &&
        accel_norm < (1.0f + cfg.accel_tol_g)) {
        float recip = 1.0f / accel_norm;
        float vx, vy, vz;   /* gravity direction estimated from quaternion */
        float ex, ey, ez;   /* attitude error (cross product) */

        if (!seeded) {
            seed_from_accel(ax, ay, az);
        }

        ax *= recip;
        ay *= recip;
        az *= recip;

        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        if (two_ki > 0.0f) {
            integ_x += two_ki * ex * dt;
            integ_y += two_ki * ey * dt;
            integ_z += two_ki * ez * dt;
            gx += integ_x;
            gy += integ_y;
            gz += integ_z;
        }
        gx += two_kp * ex;
        gy += two_kp * ey;
        gz += two_kp * ez;
    } else {
        /* Accel is not trustworthy: gyro integration only this step */
        reject_count++;
    }

    /* Magnetic heading correction (9-axis mode only) */
    if (mag_uT != NULL && two_km > 0.0f) {
        float mx = mag_uT[0], my = mag_uT[1], mz = mag_uT[2];
        float norm = sqrtf(mx * mx + my * my + mz * mz);

        if (norm > ATTITUDE_MAG_MIN_UT && norm < ATTITUDE_MAG_MAX_UT) {
            float recip = 1.0f / norm;
            float roll, pitch;
            float cf, sf, cp, sp;
            float m1x, m1y, m1z, mhx, mhy;
            float heading_meas, heading_est, e_yaw;

            mx *= recip;
            my *= recip;
            mz *= recip;

            /* Tilt compensation: remove roll/pitch so the horizontal
             * components only depend on yaw. The roll/pitch come from
             * the quaternion, which the accel keeps level. */
            quat_get_roll_pitch(&roll, &pitch);
            cf = cosf(roll);
            sf = sinf(roll);
            cp = cosf(pitch);
            sp = sinf(pitch);

            /* m1 = Ry(-pitch) * m */
            m1x = cp * mx - sp * mz;
            m1y = my;
            m1z = sp * mx + cp * mz;
            /* m_h = Rx(-roll) * m1 */
            mhx = m1x;
            mhy = cf * m1y + sf * m1z;

            heading_meas = atan2f(-mhy, mhx);
            heading_est = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                 1.0f - 2.0f * (q2 * q2 + q3 * q3));

            e_yaw = wrap_pi(heading_meas - heading_est);
            gz += two_km * e_yaw;
        } else {
            /* Not a believable Earth field: skip heading correction */
            mag_reject_count++;
        }
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;

    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;

    quat_normalize();
}

/* Shared input validation + bias handling for both update functions.
 * Returns the bias-corrected rates in g_out, 0 on success. */
static int prep_gyro(const float gyro_radps[3], const float accel_g[3],
                     float dt_s, float g_out[3])
{
    if (!inited) {
        return -EACCES;
    }
    if (gyro_radps == NULL || accel_g == NULL) {
        return -EINVAL;
    }
    if (!vec3_ok(gyro_radps) || !vec3_ok(accel_g) || !float_ok(dt_s)) {
        return -EINVAL;
    }
    if (dt_s <= 0.0f || dt_s > ATTITUDE_MAX_DT_S) {
        return -EINVAL;
    }

    /* Stationary bias re-learning: the MPU6050 zero-rate offset drifts
     * as the chip warms up after power-on (up to ~0.2 dps/degC), so a
     * one-shot calibration at boot goes stale within minutes. While
     * every axis stays below the quiet threshold, accumulate samples;
     * once the board has been quiet long enough, pull the bias toward
     * the measured mean. */
    bool quiet = fabsf(gyro_radps[0]) < ATTITUDE_QUIET_RADPS &&
                 fabsf(gyro_radps[1]) < ATTITUDE_QUIET_RADPS &&
                 fabsf(gyro_radps[2]) < ATTITUDE_QUIET_RADPS;
    if (quiet) {
        quiet_sum[0] += gyro_radps[0];
        quiet_sum[1] += gyro_radps[1];
        quiet_sum[2] += gyro_radps[2];
        quiet_time += dt_s;
        if (quiet_time >= ATTITUDE_QUIET_TIME_S) {
            float alpha = 0.5f;
            gyro_bias[0] += alpha * (quiet_sum[0] / quiet_time - gyro_bias[0]);
            gyro_bias[1] += alpha * (quiet_sum[1] / quiet_time - gyro_bias[1]);
            gyro_bias[2] += alpha * (quiet_sum[2] / quiet_time - gyro_bias[2]);
            quiet_sum[0] = quiet_sum[1] = quiet_sum[2] = 0.0f;
            quiet_time = 0.0f;
        }
    } else {
        quiet_sum[0] = quiet_sum[1] = quiet_sum[2] = 0.0f;
        quiet_time = 0.0f;
    }

    /* Subtract the bias (startup measurement + stationary re-learning) */
    g_out[0] = gyro_radps[0] - gyro_bias[0];
    g_out[1] = gyro_radps[1] - gyro_bias[1];
    g_out[2] = gyro_radps[2] - gyro_bias[2];
    return 0;
}

/* ---- Public API ---- */

int attitude_init(const struct attitude_cfg *c)
{
    if (c == NULL) {
        return -EINVAL;
    }
    if (!float_ok(c->kp) || !float_ok(c->ki) ||
        !float_ok(c->accel_tol_g) || !float_ok(c->km)) {
        return -EINVAL;
    }
    if (c->kp < 0.0f || c->ki < 0.0f || c->km < 0.0f) {
        return -EINVAL;
    }
    if (c->accel_tol_g <= 0.0f || c->accel_tol_g >= 1.0f) {
        return -EINVAL;
    }

    cfg = *c;
    two_kp = 2.0f * cfg.kp;
    two_ki = 2.0f * cfg.ki;
    two_km = 2.0f * cfg.km;

    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    seeded = false;
    integ_x = integ_y = integ_z = 0.0f;
    gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
    quiet_sum[0] = quiet_sum[1] = quiet_sum[2] = 0.0f;
    quiet_time = 0.0f;
    reject_count = 0;
    mag_reject_count = 0;
    inited = true;

    LOG_INF("attitude filter ready: kp=%d%%, ki=%d%%, accel_tol=%d%%, "
            "km=%d%%",
            (int)(cfg.kp * 100.0f), (int)(cfg.ki * 100.0f),
            (int)(cfg.accel_tol_g * 100.0f), (int)(cfg.km * 100.0f));
    return 0;
}

int attitude_update(const float gyro_radps[3], const float accel_g[3],
                    float dt_s)
{
    float g[3];
    int ret = prep_gyro(gyro_radps, accel_g, dt_s, g);

    if (ret) {
        return ret;
    }
    mahony_step(g[0], g[1], g[2],
                accel_g[0], accel_g[1], accel_g[2], dt_s, NULL);
    return 0;
}

int attitude_update_mag(const float gyro_radps[3], const float accel_g[3],
                        const float mag_uT[3], float dt_s)
{
    float g[3];
    int ret;

    if (mag_uT == NULL || !vec3_ok(mag_uT)) {
        return -EINVAL;
    }
    ret = prep_gyro(gyro_radps, accel_g, dt_s, g);
    if (ret) {
        return ret;
    }
    mahony_step(g[0], g[1], g[2],
                accel_g[0], accel_g[1], accel_g[2], dt_s, mag_uT);
    return 0;
}

int attitude_set_gyro_bias(const float bias_radps[3])
{
    if (!inited) {
        return -EACCES;
    }
    if (bias_radps == NULL || !vec3_ok(bias_radps)) {
        return -EINVAL;
    }
    gyro_bias[0] = bias_radps[0];
    gyro_bias[1] = bias_radps[1];
    gyro_bias[2] = bias_radps[2];
    return 0;
}

int attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!inited) {
        return -EACCES;
    }
    if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) {
        return -EINVAL;
    }

    float roll, pitch;
    quat_get_roll_pitch(&roll, &pitch);

    *roll_deg = roll * ATTITUDE_RAD_TO_DEG;
    *pitch_deg = pitch * ATTITUDE_RAD_TO_DEG;
    *yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATTITUDE_RAD_TO_DEG;
    return 0;
}

int attitude_get_quat(float q[4])
{
    if (!inited) {
        return -EACCES;
    }
    if (q == NULL) {
        return -EINVAL;
    }
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
    return 0;
}

uint32_t attitude_get_reject_count(void)
{
    return reject_count;
}

uint32_t attitude_get_mag_reject_count(void)
{
    return mag_reject_count;
}
