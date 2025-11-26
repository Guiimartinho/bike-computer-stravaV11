/**
 * @file segment.h
 * @brief Strava Segment management for stravaV10
 *
 * Handles loading, tracking, and competing on Strava segments.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef MODEL_SEGMENT_H
#define MODEL_SEGMENT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** Segment activation distance in meters */
#define SEG_ACTIVATE_DIST       50.0f

/** Segment deactivation distance in meters */
#define SEG_DEACTIVATE_DIST     100.0f

/** Maximum segment name length */
#define SEG_NAME_MAX_LEN        13U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Segment point data (stored in file) */
typedef struct {
    float lat;          /**< Latitude in degrees */
    float lon;          /**< Longitude in degrees */
    float alt;          /**< Altitude in meters */
    float dist;         /**< Cumulative distance */
    float time;         /**< Reference time at this point */
} seg_point_t;

/** Segment file header */
typedef struct {
    char name[SEG_NAME_MAX_LEN];    /**< Segment name */
    uint16_t num_points;            /**< Number of points */
    float total_dist;               /**< Total distance */
    float total_time;               /**< Reference total time */
    float total_elev;               /**< Total elevation gain */
    uint32_t checksum;              /**< File checksum */
} seg_header_t;

/** Segment status callback */
typedef void (*seg_status_callback_t)(const segment_t *seg);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize segment manager
 * @return APP_OK on success, error code otherwise
 */
app_err_t segment_init(void);

/**
 * @brief Load segments from SD card
 * @return Number of segments loaded, or negative error code
 */
int segment_load_all(void);

/**
 * @brief Update segments with current position
 * @param loc Current location
 * @return APP_OK on success, error code otherwise
 */
app_err_t segment_update(const loc_data_t *loc);

/**
 * @brief Get active segment count
 * @return Number of currently active segments
 */
uint8_t segment_get_active_count(void);

/**
 * @brief Get segment by index
 * @param index Segment index
 * @param seg Pointer to store segment data
 * @return APP_OK on success, error code otherwise
 */
app_err_t segment_get(uint8_t index, segment_t *seg);

/**
 * @brief Get best active segment (for display)
 * @param seg Pointer to store segment data
 * @return APP_OK on success, APP_ERR_NOT_FOUND if none active
 */
app_err_t segment_get_best(segment_t *seg);

/**
 * @brief Get all active segments
 * @param segs Array to store segments
 * @param max_count Maximum number of segments
 * @return Number of active segments
 */
uint8_t segment_get_active(segment_t *segs, uint8_t max_count);

/**
 * @brief Register callback for segment status changes
 * @param callback Function to call on status change
 * @return APP_OK on success, error code otherwise
 */
app_err_t segment_register_callback(seg_status_callback_t callback);

/**
 * @brief Get distance to nearest segment start
 * @return Distance in meters, or -1 if no segments
 */
float segment_get_nearest_distance(void);

/**
 * @brief Get total loaded segment count
 * @return Number of loaded segments
 */
uint16_t segment_get_total_count(void);

/**
 * @brief Check if any segment is active
 * @return true if at least one segment is active
 */
bool segment_is_any_active(void);

/**
 * @brief Reset all segment states
 */
void segment_reset_all(void);

/**
 * @brief Unload all segments
 */
void segment_unload_all(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_SEGMENT_H */
