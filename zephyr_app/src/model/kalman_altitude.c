/**
 * @file kalman_altitude.c
 * @brief 3-State Kalman filter implementation for altitude/slope estimation
 *
 * Based on original Attitude.cpp implementation:
 *
 * State: [elevation, pitch, alpha_zero]
 *
 * Model:
 *   d(elevation)/dt = speed * tan(slope)
 *   slope = pitch - alpha_zero
 *   d(elevation) = (pitch - alpha_zero) * dl
 *   dl = speed * dt
 *
 * State transition matrix A:
 *   | 1  dl -dl |
 *   | 0   1   0 |
 *   | 0   0   1 |
 *
 * Observation matrix C:
 *   | 1  0  0 |  (barometer measures elevation)
 *   | 0  1  0 |  (accelerometer measures pitch)
 */

#include <string.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "model/kalman_altitude.h"

LOG_MODULE_REGISTER(kalman_altitude, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Minimum speed for Kalman update (m/s) */
#define MIN_SPEED_FOR_UPDATE    1.5f

/** Default process noise values (from original) */
#define DEFAULT_Q_ELEVATION     0.03f
#define DEFAULT_Q_PITCH         0.10f
#define DEFAULT_Q_ALPHA         0.0002f

/** Default measurement noise values (from original) */
#define DEFAULT_R_BARO          1000.0f
#define DEFAULT_R_ACCEL         600.0f

/** Initial covariance */
#define INITIAL_P_VALUE         900.0f

/** Covariance bounds */
#define P_MIN                   1e-15f
#define P_MAX                   1e12f

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Perform time update (prediction step)
 */
static void time_update(kalman_altitude_t *kf, float dl)
{
    /* Update state transition matrix A with current dl
     * A = | 1  dl -dl |
     *     | 0   1   0 |
     *     | 0   0   1 |
     */
    udmat_identity(&kf->mat_a, 1.0f);
    udmat_set(&kf->mat_a, 0, 1, dl);
    udmat_set(&kf->mat_a, 0, 2, -dl);

    /* Predict state: x_pred = A * x */
    (void)udmat_mul(&kf->mat_a, &kf->mat_x, &kf->mat_x_pred);

    /* Predict covariance: P_pred = A * P * A' + Q */
    udmatrix_t at;
    udmat_transpose(&kf->mat_a, &at);

    udmatrix_t ap;
    (void)udmat_mul(&kf->mat_a, &kf->mat_p, &ap);
    (void)udmat_mul(&ap, &at, &kf->mat_p_pred);
    (void)udmat_add(&kf->mat_p_pred, &kf->mat_q, &kf->mat_p_pred);

    /* Bound covariance */
    udmat_bound(&kf->mat_p_pred, P_MIN, P_MAX);
}

/**
 * @brief Perform measurement update (correction step)
 */
static void measurement_update(kalman_altitude_t *kf, float z_baro, float z_pitch)
{
    /* Observation matrix C:
     * C = | 1  0  0 |
     *     | 0  1  0 |
     */
    udmat_zeros(&kf->mat_c);
    udmat_set(&kf->mat_c, 0, 0, 1.0f);
    udmat_set(&kf->mat_c, 1, 1, 1.0f);

    /* Calculate Kalman gain: K = P_pred * C' * (C * P_pred * C' + R)^-1 */

    /* C' (transpose) */
    udmatrix_t ct;
    udmat_transpose(&kf->mat_c, &ct);

    /* P_pred * C' */
    udmatrix_t pct;
    (void)udmat_mul(&kf->mat_p_pred, &ct, &pct);

    /* C * P_pred * C' */
    udmatrix_t cpct;
    (void)udmat_mul(&kf->mat_c, &pct, &cpct);

    /* C * P_pred * C' + R */
    udmatrix_t cpctr;
    (void)udmat_add(&cpct, &kf->mat_r, &cpctr);

    /* (C * P_pred * C' + R)^-1 */
    udmatrix_t cpctr_inv;
    if (!udmat_invert(&cpctr, &cpctr_inv)) {
        LOG_WRN("Matrix inversion failed");
        return;
    }

    /* K = P_pred * C' * inv */
    (void)udmat_mul(&pct, &cpctr_inv, &kf->mat_k);

    /* Create measurement vector z */
    udmatrix_t mat_z;
    udmat_init(&mat_z, KALT_OBS_DIM, 1U);
    udmat_set(&mat_z, 0, 0, z_baro);
    udmat_set(&mat_z, 1, 0, z_pitch);

    /* Innovation: y = z - C * x_pred */
    udmatrix_t cx;
    (void)udmat_mul(&kf->mat_c, &kf->mat_x_pred, &cx);

    udmatrix_t innov;
    (void)udmat_sub(&mat_z, &cx, &innov);

    /* Update state: x = x_pred + K * y */
    (void)udmat_mul(&kf->mat_k, &innov, &kf->mat_ki);
    (void)udmat_add(&kf->mat_x_pred, &kf->mat_ki, &kf->mat_x);

    /* Update covariance: P = (I - K * C) * P_pred */
    udmatrix_t kc;
    (void)udmat_mul(&kf->mat_k, &kf->mat_c, &kc);

    udmatrix_t eye;
    udmat_init(&eye, KALT_STATE_DIM, KALT_STATE_DIM);
    udmat_identity(&eye, 1.0f);

    udmatrix_t ikc;
    (void)udmat_sub(&eye, &kc, &ikc);

    (void)udmat_mul(&ikc, &kf->mat_p_pred, &kf->mat_p);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void kalman_altitude_init(kalman_altitude_t *kf, float initial_alt)
{
    if (kf == NULL) {
        return;
    }

    /* Clear structure */
    (void)memset(kf, 0, sizeof(kalman_altitude_t));

    /* Initialize state vector */
    udmat_init(&kf->mat_x, KALT_STATE_DIM, 1U);
    udmat_set(&kf->mat_x, 0, 0, initial_alt);  /* elevation */
    udmat_set(&kf->mat_x, 1, 0, 0.0f);         /* pitch */
    udmat_set(&kf->mat_x, 2, 0, 0.0f);         /* alpha_zero */

    udmat_init(&kf->mat_x_pred, KALT_STATE_DIM, 1U);

    /* Initialize matrices */
    udmat_init(&kf->mat_a, KALT_STATE_DIM, KALT_STATE_DIM);
    udmat_identity(&kf->mat_a, 1.0f);

    udmat_init(&kf->mat_b, KALT_STATE_DIM, KALT_STATE_DIM);
    udmat_zeros(&kf->mat_b);

    udmat_init(&kf->mat_c, KALT_OBS_DIM, KALT_STATE_DIM);
    udmat_zeros(&kf->mat_c);
    udmat_set(&kf->mat_c, 0, 0, 1.0f);
    udmat_set(&kf->mat_c, 1, 1, 1.0f);

    /* Initialize covariance matrices */
    udmat_init(&kf->mat_p, KALT_STATE_DIM, KALT_STATE_DIM);
    udmat_ones(&kf->mat_p, INITIAL_P_VALUE);

    udmat_init(&kf->mat_p_pred, KALT_STATE_DIM, KALT_STATE_DIM);

    /* Process noise Q */
    udmat_init(&kf->mat_q, KALT_STATE_DIM, KALT_STATE_DIM);
    udmat_zeros(&kf->mat_q);
    udmat_set(&kf->mat_q, 0, 0, DEFAULT_Q_ELEVATION);
    udmat_set(&kf->mat_q, 1, 1, DEFAULT_Q_PITCH);
    udmat_set(&kf->mat_q, 2, 2, DEFAULT_Q_ALPHA);

    /* Measurement noise R */
    udmat_init(&kf->mat_r, KALT_OBS_DIM, KALT_OBS_DIM);
    udmat_identity(&kf->mat_r, 1.0f);
    udmat_set(&kf->mat_r, 0, 0, DEFAULT_R_BARO);
    udmat_set(&kf->mat_r, 1, 1, DEFAULT_R_ACCEL);

    /* Kalman gain */
    udmat_init(&kf->mat_k, KALT_STATE_DIM, KALT_OBS_DIM);
    udmat_init(&kf->mat_ki, KALT_STATE_DIM, 1U);

    kf->is_init = true;
    kf->last_update_ms = 0U;

    LOG_INF("Kalman altitude filter initialized (alt=%.1f)", (double)initial_alt);
}

void kalman_altitude_reset(kalman_altitude_t *kf)
{
    if (kf == NULL) {
        return;
    }

    kf->is_init = false;
    kf->last_update_ms = 0U;
}

bool kalman_altitude_update(kalman_altitude_t *kf,
                            const kalman_alt_feed_t *feed,
                            kalman_alt_output_t *output)
{
    if ((kf == NULL) || (feed == NULL)) {
        return false;
    }

    /* Initialize if not done */
    if (!kf->is_init) {
        kalman_altitude_init(kf, feed->baro_altitude);
    }

    /* Skip if speed too low */
    if (feed->speed_ms < MIN_SPEED_FOR_UPDATE) {
        kf->last_update_ms = feed->timestamp_ms;

        if (output != NULL) {
            output->elevation = udmat_get(&kf->mat_x, 0, 0);
            output->pitch = udmat_get(&kf->mat_x, 1, 0);
            output->alpha_zero = udmat_get(&kf->mat_x, 2, 0);
            output->slope = 0.0f;
            output->vit_asc = 0.0f;
        }

        return false;
    }

    /* Calculate dl = speed * dt */
    float dt = 0.001f * (float)(feed->timestamp_ms - kf->last_update_ms);
    if (dt <= 0.0f) {
        dt = 0.1f;
    }

    float dl = feed->speed_ms * dt;

    /* Prediction step */
    time_update(kf, dl);

    /* Measurement update */
    measurement_update(kf, feed->baro_altitude, feed->pitch_rad);

    kf->last_update_ms = feed->timestamp_ms;

    /* Extract results */
    if (output != NULL) {
        output->elevation = udmat_get(&kf->mat_x, 0, 0);
        output->pitch = udmat_get(&kf->mat_x, 1, 0);
        output->alpha_zero = udmat_get(&kf->mat_x, 2, 0);
        output->slope = output->pitch - output->alpha_zero;
        output->vit_asc = output->slope * feed->speed_ms;
    }

    LOG_DBG("Kalman: ele=%.1f, pitch=%.3f, a0=%.3f",
            (double)udmat_get(&kf->mat_x, 0, 0),
            (double)udmat_get(&kf->mat_x, 1, 0),
            (double)udmat_get(&kf->mat_x, 2, 0));

    return true;
}

float kalman_altitude_get_elevation(const kalman_altitude_t *kf)
{
    if ((kf == NULL) || !kf->is_init) {
        return 0.0f;
    }

    return udmat_get(&kf->mat_x, 0, 0);
}

int8_t kalman_altitude_get_slope(const kalman_altitude_t *kf)
{
    if ((kf == NULL) || !kf->is_init) {
        return 0;
    }

    float pitch = udmat_get(&kf->mat_x, 1, 0);
    float alpha_zero = udmat_get(&kf->mat_x, 2, 0);
    float slope = pitch - alpha_zero;

    /* Convert to percent and clamp */
    float slope_pct = 100.0f * slope;

    if (slope_pct > 45.0f) {
        slope_pct = 45.0f;
    } else if (slope_pct < -45.0f) {
        slope_pct = -45.0f;
    }

    return (int8_t)slope_pct;
}

float kalman_altitude_get_vspeed(const kalman_altitude_t *kf, float speed_ms)
{
    if ((kf == NULL) || !kf->is_init) {
        return 0.0f;
    }

    float pitch = udmat_get(&kf->mat_x, 1, 0);
    float alpha_zero = udmat_get(&kf->mat_x, 2, 0);
    float slope = pitch - alpha_zero;

    return slope * speed_ms;
}

bool kalman_altitude_is_init(const kalman_altitude_t *kf)
{
    if (kf == NULL) {
        return false;
    }

    return kf->is_init;
}

void kalman_altitude_set_process_noise(kalman_altitude_t *kf,
                                       float q_elevation,
                                       float q_pitch,
                                       float q_alpha)
{
    if (kf == NULL) {
        return;
    }

    udmat_zeros(&kf->mat_q);
    udmat_set(&kf->mat_q, 0, 0, q_elevation);
    udmat_set(&kf->mat_q, 1, 1, q_pitch);
    udmat_set(&kf->mat_q, 2, 2, q_alpha);
}

void kalman_altitude_set_measurement_noise(kalman_altitude_t *kf,
                                           float r_baro,
                                           float r_accel)
{
    if (kf == NULL) {
        return;
    }

    udmat_zeros(&kf->mat_r);
    udmat_set(&kf->mat_r, 0, 0, r_baro);
    udmat_set(&kf->mat_r, 1, 1, r_accel);
}
