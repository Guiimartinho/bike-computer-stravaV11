/**
 * @file baro_drift.h
 * @brief Barometer drift correction using GPS altitude
 *
 * High-pass filter to compensate barometer drift over time.
 * Based on original Attitude.cpp filterElevation() implementation.
 *
 * The barometer provides high-frequency altitude changes accurately but
 * drifts over time. GPS provides accurate absolute altitude but with noise.
 * This filter combines both to get accurate altitude with good responsiveness.
 */

#ifndef MODEL_BARO_DRIFT_H
#define MODEL_BARO_DRIFT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Time constant for drift filter (seconds) - from original */
#define BARO_DRIFT_TAU_SECONDS      800.0f

/** Minimum GPS fix age to use for correction (ms) */
#define BARO_DRIFT_MIN_GPS_AGE_MS   5000U

/** Maximum GPS fix age to consider valid (ms) */
#define BARO_DRIFT_MAX_GPS_AGE_MS   30000U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Barometer drift correction state
 */
typedef struct {
    float correction;           /**< Current correction value (m) */
    float alt_div;              /**< Filtered altitude difference */
    float tau;                  /**< Time constant (seconds) */
    uint32_t last_gps_time;     /**< Last GPS update timestamp */
    uint32_t last_update_time;  /**< Last filter update timestamp */
    bool is_initialized;        /**< Initialization flag */
} baro_drift_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize barometer drift correction
 * @param bd Pointer to drift correction state
 */
void baro_drift_init(baro_drift_t *bd);

/**
 * @brief Reset drift correction
 * @param bd Pointer to drift correction state
 */
void baro_drift_reset(baro_drift_t *bd);

/**
 * @brief Update drift correction with GPS altitude
 *
 * Call this when a new GPS fix is received.
 *
 * @param bd Pointer to drift correction state
 * @param baro_alt Current barometer altitude (m)
 * @param gps_alt GPS altitude (m)
 * @param timestamp_ms Current timestamp (ms)
 */
void baro_drift_update_gps(baro_drift_t *bd, float baro_alt, float gps_alt,
                           uint32_t timestamp_ms);

/**
 * @brief Get corrected barometer altitude
 *
 * Apply the drift correction to a raw barometer reading.
 *
 * @param bd Pointer to drift correction state
 * @param baro_alt Raw barometer altitude (m)
 * @return Corrected altitude (m)
 */
float baro_drift_correct(const baro_drift_t *bd, float baro_alt);

/**
 * @brief Get current correction value
 * @param bd Pointer to drift correction state
 * @return Current correction in meters
 */
float baro_drift_get_correction(const baro_drift_t *bd);

/**
 * @brief Set time constant
 * @param bd Pointer to drift correction state
 * @param tau_seconds Time constant in seconds
 */
void baro_drift_set_tau(baro_drift_t *bd, float tau_seconds);

/**
 * @brief Check if drift correction is active
 * @param bd Pointer to drift correction state
 * @return true if correction is being applied
 */
bool baro_drift_is_active(const baro_drift_t *bd);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_BARO_DRIFT_H */
