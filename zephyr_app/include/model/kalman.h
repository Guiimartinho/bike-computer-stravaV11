/**
 * @file kalman.h
 * @brief Kalman filter for sensor fusion
 *
 * Simple 1D Kalman filter for smoothing sensor data.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef MODEL_KALMAN_H
#define MODEL_KALMAN_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Kalman filter state structure */
typedef struct {
    float x;            /**< State estimate */
    float p;            /**< Estimate covariance */
    float q;            /**< Process noise covariance */
    float r;            /**< Measurement noise covariance */
    float k;            /**< Kalman gain (computed) */
    bool initialized;   /**< Initialization flag */
} kalman_state_t;

/** 2D Kalman filter for position (lat/lon) */
typedef struct {
    kalman_state_t lat;     /**< Latitude filter */
    kalman_state_t lon;     /**< Longitude filter */
    kalman_state_t alt;     /**< Altitude filter */
    kalman_state_t speed;   /**< Speed filter */
} kalman_position_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize Kalman filter with default parameters
 * @param state Pointer to Kalman state
 * @param process_noise Process noise (Q)
 * @param measurement_noise Measurement noise (R)
 */
void kalman_init(kalman_state_t *state, float process_noise, float measurement_noise);

/**
 * @brief Update Kalman filter with new measurement
 * @param state Pointer to Kalman state
 * @param measurement New measurement value
 * @return Filtered estimate
 */
float kalman_update(kalman_state_t *state, float measurement);

/**
 * @brief Set Kalman filter state directly
 * @param state Pointer to Kalman state
 * @param value Initial state value
 */
void kalman_set(kalman_state_t *state, float value);

/**
 * @brief Get current state estimate
 * @param state Pointer to Kalman state
 * @return Current estimate
 */
float kalman_get(const kalman_state_t *state);

/**
 * @brief Reset Kalman filter
 * @param state Pointer to Kalman state
 */
void kalman_reset(kalman_state_t *state);

/**
 * @brief Initialize position Kalman filter
 * @param pos Pointer to position filter
 */
void kalman_position_init(kalman_position_t *pos);

/**
 * @brief Update position with new GPS data
 * @param pos Pointer to position filter
 * @param loc New location measurement
 * @return Filtered location
 */
loc_data_t kalman_position_update(kalman_position_t *pos, const loc_data_t *loc);

/**
 * @brief Reset position filter
 * @param pos Pointer to position filter
 */
void kalman_position_reset(kalman_position_t *pos);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_KALMAN_H */
