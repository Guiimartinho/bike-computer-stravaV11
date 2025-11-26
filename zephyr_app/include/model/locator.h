/**
 * @file locator.h
 * @brief Position locator and GPS history management
 *
 * Manages GPS position history and provides distance/bearing calculations.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef MODEL_LOCATOR_H
#define MODEL_LOCATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** Earth radius in meters */
#define EARTH_RADIUS_M          6371000.0f

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** History point with additional computed data */
typedef struct {
    loc_data_t loc;         /**< Location data */
    float dist_from_prev;   /**< Distance from previous point (m) */
    float bearing;          /**< Bearing from previous point (rad) */
    bool valid;             /**< Data validity */
} history_point_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize locator module
 * @return APP_OK on success, error code otherwise
 */
app_err_t locator_init(void);

/**
 * @brief Add new position to history
 * @param loc Position data to add
 * @return APP_OK on success, error code otherwise
 */
app_err_t locator_add_point(const loc_data_t *loc);

/**
 * @brief Get current position
 * @param loc Pointer to store current position
 * @return APP_OK on success, error code otherwise
 */
app_err_t locator_get_current(loc_data_t *loc);

/**
 * @brief Get history point by index
 * @param index Index (0 = most recent)
 * @param point Pointer to store history point
 * @return APP_OK on success, error code otherwise
 */
app_err_t locator_get_history(uint8_t index, history_point_t *point);

/**
 * @brief Get total distance traveled
 * @return Total distance in meters
 */
float locator_get_total_distance(void);

/**
 * @brief Get total elevation gain
 * @return Total climb in meters
 */
float locator_get_total_climb(void);

/**
 * @brief Get current speed (smoothed)
 * @return Speed in km/h
 */
float locator_get_speed(void);

/**
 * @brief Get current slope
 * @return Slope in percent
 */
int8_t locator_get_slope(void);

/**
 * @brief Get vertical speed
 * @return Vertical speed in m/s
 */
float locator_get_vspeed(void);

/**
 * @brief Calculate distance between two points
 * @param lat1 Latitude of point 1 (degrees)
 * @param lon1 Longitude of point 1 (degrees)
 * @param lat2 Latitude of point 2 (degrees)
 * @param lon2 Longitude of point 2 (degrees)
 * @return Distance in meters
 */
float locator_calc_distance(float lat1, float lon1, float lat2, float lon2);

/**
 * @brief Calculate bearing between two points
 * @param lat1 Latitude of point 1 (degrees)
 * @param lon1 Longitude of point 1 (degrees)
 * @param lat2 Latitude of point 2 (degrees)
 * @param lon2 Longitude of point 2 (degrees)
 * @return Bearing in radians (0 = North)
 */
float locator_calc_bearing(float lat1, float lon1, float lat2, float lon2);

/**
 * @brief Check if position is valid
 * @param loc Position to check
 * @return true if position is valid
 */
bool locator_is_valid_position(const loc_data_t *loc);

/**
 * @brief Reset locator state
 */
void locator_reset(void);

/**
 * @brief Get number of points in history
 * @return Number of valid history points
 */
uint8_t locator_get_history_count(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_LOCATOR_H */
