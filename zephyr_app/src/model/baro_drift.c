/**
 * @file baro_drift.c
 * @brief Barometer drift correction implementation
 *
 * Based on original Attitude.cpp filterElevation():
 *
 * The barometer drifts over time due to atmospheric pressure changes.
 * This high-pass filter uses GPS altitude as reference to remove drift
 * while preserving the barometer's high-frequency response.
 *
 * Filter equation:
 *   alt_div = tau * alt_div + (1 - tau) * (baro_alt - gps_alt)
 *   correction = alt_div
 *   corrected_alt = baro_alt - correction
 *
 * Where tau = T / (T + dt) and T = 800 seconds (from original)
 */

#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "model/baro_drift.h"

LOG_MODULE_REGISTER(baro_drift, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void baro_drift_init(baro_drift_t *bd)
{
    if (bd == NULL) {
        return;
    }

    (void)memset(bd, 0, sizeof(baro_drift_t));
    bd->tau = BARO_DRIFT_TAU_SECONDS;
    bd->is_initialized = true;

    LOG_INF("Baro drift correction initialized (tau=%.0f s)", (double)bd->tau);
}

void baro_drift_reset(baro_drift_t *bd)
{
    if (bd == NULL) {
        return;
    }

    bd->correction = 0.0f;
    bd->alt_div = 0.0f;
    bd->last_gps_time = 0U;
    bd->last_update_time = 0U;

    LOG_INF("Baro drift correction reset");
}

void baro_drift_update_gps(baro_drift_t *bd, float baro_alt, float gps_alt,
                           uint32_t timestamp_ms)
{
    if ((bd == NULL) || !bd->is_initialized) {
        return;
    }

    /* Calculate time delta */
    float dt = 1.0f;  /* Default 1 second */

    if (bd->last_update_time > 0U) {
        uint32_t delta_ms = timestamp_ms - bd->last_update_time;

        /* Sanity check on delta */
        if ((delta_ms > 0U) && (delta_ms < 60000U)) {
            dt = (float)delta_ms / 1000.0f;
        }
    }

    /* Calculate filter coefficient
     * tau_coef = T / (T + dt)
     * where T = time constant in seconds
     *
     * For T=800s and dt=1s: tau_coef = 800/801 = 0.99875
     * This gives a very slow response, as intended
     */
    float tau_coef = bd->tau / (bd->tau + dt);

    /* Calculate altitude difference (what the barometer says vs GPS) */
    float input = baro_alt - gps_alt;

    /* First update - initialize with current difference */
    if (bd->last_update_time == 0U) {
        bd->alt_div = input;
        LOG_INF("Baro drift initial offset: %.1f m", (double)input);
    } else {
        /* Apply high-pass filter
         * This slowly tracks the difference between baro and GPS
         * Over time, alt_div converges to the barometer's drift
         */
        bd->alt_div = (tau_coef * bd->alt_div) + ((1.0f - tau_coef) * input);
    }

    /* Update correction value */
    bd->correction = bd->alt_div;

    /* Update timestamps */
    bd->last_gps_time = timestamp_ms;
    bd->last_update_time = timestamp_ms;

    LOG_DBG("Baro drift: baro=%.1f, gps=%.1f, corr=%.2f",
            (double)baro_alt, (double)gps_alt, (double)bd->correction);
}

float baro_drift_correct(const baro_drift_t *bd, float baro_alt)
{
    if ((bd == NULL) || !bd->is_initialized) {
        return baro_alt;
    }

    /* Apply correction: subtract the accumulated drift */
    return baro_alt - bd->correction;
}

float baro_drift_get_correction(const baro_drift_t *bd)
{
    if ((bd == NULL) || !bd->is_initialized) {
        return 0.0f;
    }

    return bd->correction;
}

void baro_drift_set_tau(baro_drift_t *bd, float tau_seconds)
{
    if (bd == NULL) {
        return;
    }

    /* Clamp to reasonable range */
    if (tau_seconds < 60.0f) {
        tau_seconds = 60.0f;
    } else if (tau_seconds > 3600.0f) {
        tau_seconds = 3600.0f;
    }

    bd->tau = tau_seconds;
    LOG_INF("Baro drift tau set to %.0f s", (double)tau_seconds);
}

bool baro_drift_is_active(const baro_drift_t *bd)
{
    if ((bd == NULL) || !bd->is_initialized) {
        return false;
    }

    /* Active if we have received at least one GPS update */
    return (bd->last_gps_time > 0U);
}
