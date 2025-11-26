/**
 * @file kalman_altitude.h
 * @brief 3-State Kalman filter for altitude/slope estimation
 * @note Follows MISRA C:2012 guidelines
 *
 * State vector: [elevation, pitch, alpha_zero]
 * - elevation: Filtered altitude in meters
 * - pitch: Pitch angle from accelerometer
 * - alpha_zero: Accelerometer mounting offset (bias)
 *
 * Model: d(elevation)/dt = speed * tan(slope)
 *        slope = pitch - alpha_zero
 */

#ifndef MODEL_KALMAN_ALTITUDE_H_
#define MODEL_KALMAN_ALTITUDE_H_

#include <stdint.h>
#include <stdbool.h>
#include "model/udmatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** State dimension */
#define KALT_STATE_DIM      3U

/** Observation dimension */
#define KALT_OBS_DIM        2U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Altitude Kalman filter state
 */
typedef struct {
    /* State vector X = [elevation, pitch, alpha_zero] */
    udmatrix_t mat_x;       /**< State estimate */
    udmatrix_t mat_x_pred;  /**< Predicted state */

    /* System matrices */
    udmatrix_t mat_a;       /**< State transition matrix */
    udmatrix_t mat_b;       /**< Control input matrix */
    udmatrix_t mat_c;       /**< Observation matrix */

    /* Covariance matrices */
    udmatrix_t mat_p;       /**< Error covariance */
    udmatrix_t mat_p_pred;  /**< Predicted covariance */
    udmatrix_t mat_q;       /**< Process noise covariance */
    udmatrix_t mat_r;       /**< Measurement noise covariance */

    /* Kalman gain */
    udmatrix_t mat_k;       /**< Kalman gain */
    udmatrix_t mat_ki;      /**< K * innovation */

    /* Initialization flag */
    bool is_init;

    /* Last update timestamp */
    uint32_t last_update_ms;
} kalman_altitude_t;

/**
 * @brief Kalman filter feed (measurements)
 */
typedef struct {
    float baro_altitude;    /**< Barometer altitude (m) */
    float pitch_rad;        /**< Pitch from accelerometer (rad) */
    float speed_ms;         /**< Current speed (m/s) */
    uint32_t timestamp_ms;  /**< Current timestamp */
} kalman_alt_feed_t;

/**
 * @brief Kalman filter output
 */
typedef struct {
    float elevation;        /**< Filtered elevation (m) */
    float pitch;            /**< Filtered pitch (rad) */
    float alpha_zero;       /**< Mounting offset (rad) */
    float slope;            /**< Estimated slope = pitch - alpha_zero */
    float vit_asc;          /**< Vertical speed (m/s) */
} kalman_alt_output_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize altitude Kalman filter
 * @param kf Pointer to filter state
 * @param initial_alt Initial altitude estimate
 */
void kalman_altitude_init(kalman_altitude_t *kf, float initial_alt);

/**
 * @brief Reset Kalman filter
 * @param kf Pointer to filter state
 */
void kalman_altitude_reset(kalman_altitude_t *kf);

/**
 * @brief Feed new measurements to filter
 * @param kf Pointer to filter state
 * @param feed Pointer to measurements
 * @param output Pointer to store results
 * @return true if update performed, false if skipped
 */
bool kalman_altitude_update(kalman_altitude_t *kf,
                            const kalman_alt_feed_t *feed,
                            kalman_alt_output_t *output);

/**
 * @brief Get current filtered elevation
 * @param kf Pointer to filter state
 * @return Filtered elevation in meters
 */
float kalman_altitude_get_elevation(const kalman_altitude_t *kf);

/**
 * @brief Get current estimated slope
 * @param kf Pointer to filter state
 * @return Slope in percent (0-100%)
 */
int8_t kalman_altitude_get_slope(const kalman_altitude_t *kf);

/**
 * @brief Get vertical speed
 * @param kf Pointer to filter state
 * @param speed_ms Current horizontal speed
 * @return Vertical speed in m/s
 */
float kalman_altitude_get_vspeed(const kalman_altitude_t *kf, float speed_ms);

/**
 * @brief Check if filter is initialized
 * @param kf Pointer to filter state
 * @return true if initialized
 */
bool kalman_altitude_is_init(const kalman_altitude_t *kf);

/**
 * @brief Set process noise parameters
 * @param kf Pointer to filter state
 * @param q_elevation Elevation noise
 * @param q_pitch Pitch noise
 * @param q_alpha Alpha zero noise
 */
void kalman_altitude_set_process_noise(kalman_altitude_t *kf,
                                       float q_elevation,
                                       float q_pitch,
                                       float q_alpha);

/**
 * @brief Set measurement noise parameters
 * @param kf Pointer to filter state
 * @param r_baro Barometer noise
 * @param r_accel Accelerometer noise
 */
void kalman_altitude_set_measurement_noise(kalman_altitude_t *kf,
                                           float r_baro,
                                           float r_accel);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_KALMAN_ALTITUDE_H_ */
