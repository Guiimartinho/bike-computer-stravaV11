/**
 * @file parcours.h
 * @brief Parcours (Route) management for navigation
 *
 * Handles loading and following of GPX routes for navigation.
 */

#ifndef MODEL_PARCOURS_H
#define MODEL_PARCOURS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"
#include "model/liste_points.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum points per parcours */
#define PARCOURS_MAX_POINTS     500U

/** Maximum name length */
#define PARCOURS_NAME_LEN       32U

/** Off-route threshold in meters */
#define PARCOURS_OFF_ROUTE_M    50.0f

/** Point proximity threshold in meters */
#define PARCOURS_NEAR_POINT_M   30.0f

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Parcours state */
typedef enum {
    PARCOURS_STATE_IDLE = 0,    /**< No parcours loaded */
    PARCOURS_STATE_LOADED,      /**< Parcours loaded, not started */
    PARCOURS_STATE_ACTIVE,      /**< Actively navigating */
    PARCOURS_STATE_FINISHED,    /**< Reached end of route */
    PARCOURS_STATE_OFF_ROUTE    /**< Too far from route */
} parcours_state_t;

/** Parcours info structure */
typedef struct {
    char name[PARCOURS_NAME_LEN];   /**< Route name */
    uint16_t num_points;            /**< Number of points */
    float total_distance;           /**< Total distance in meters */
    float total_climb;              /**< Total elevation gain */
    bool valid;                     /**< Is parcours valid */
} parcours_info_t;

/** Navigation info structure */
typedef struct {
    float dist_to_route;        /**< Distance to nearest point on route (m) */
    float dist_remaining;       /**< Distance remaining to end (m) */
    float dist_completed;       /**< Distance completed (m) */
    float pct_complete;         /**< Percentage complete (0-100) */
    float bearing;              /**< Bearing to next point (degrees) */
    float altitude_next;        /**< Altitude of next point (m) */
    uint16_t current_idx;       /**< Current point index */
    bool on_route;              /**< True if within threshold of route */
} nav_info_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize parcours module
 * @return APP_OK on success
 */
app_err_t parcours_init(void);

/**
 * @brief Load parcours from file
 * @param filename Path to GPX/CRS file
 * @return APP_OK on success
 */
app_err_t parcours_load(const char *filename);

/**
 * @brief Unload current parcours
 */
void parcours_unload(void);

/**
 * @brief Start navigation
 * @return APP_OK on success
 */
app_err_t parcours_start(void);

/**
 * @brief Stop navigation
 */
void parcours_stop(void);

/**
 * @brief Update navigation with current position
 * @param lat Current latitude
 * @param lon Current longitude
 * @param alt Current altitude
 */
void parcours_update(float lat, float lon, float alt);

/**
 * @brief Get parcours state
 * @return Current state
 */
parcours_state_t parcours_get_state(void);

/**
 * @brief Get parcours info
 * @param info Pointer to info structure to fill
 * @return APP_OK on success
 */
app_err_t parcours_get_info(parcours_info_t *info);

/**
 * @brief Get navigation info
 * @param nav Pointer to nav info structure to fill
 * @return APP_OK on success
 */
app_err_t parcours_get_nav_info(nav_info_t *nav);

/**
 * @brief Check if parcours is loaded
 * @return true if loaded
 */
bool parcours_is_loaded(void);

/**
 * @brief Check if currently navigating
 * @return true if active
 */
bool parcours_is_active(void);

/**
 * @brief Get point at index
 * @param index Point index
 * @return Pointer to point or NULL
 */
const point_t *parcours_get_point(uint16_t index);

/**
 * @brief Get number of points
 * @return Number of points
 */
uint16_t parcours_get_num_points(void);

/**
 * @brief Get current point index (nearest passed point)
 * @return Current index
 */
uint16_t parcours_get_current_index(void);

/**
 * @brief Get distance to next waypoint
 * @return Distance in meters
 */
float parcours_get_dist_to_next(void);

/**
 * @brief Get bearing to next waypoint
 * @return Bearing in degrees (0-360)
 */
float parcours_get_bearing_to_next(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_PARCOURS_H */
