/**
 * @file liste_points.h
 * @brief Point list management for segment tracking
 * @note Follows MISRA C:2012 guidelines
 */

#ifndef MODEL_LISTE_POINTS_H_
#define MODEL_LISTE_POINTS_H_

#include <stdint.h>
#include <stdbool.h>
#include "model/vecteur.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum points in user history */
#define LISTE_MAX_HISTORY       40U

/** Maximum points in a segment */
#define LISTE_MAX_SEGMENT       500U

/** Number of points for recording (rolling buffer) */
#define NB_RECORDING            40U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Point list structure (circular buffer)
 */
typedef struct {
    point_t points[LISTE_MAX_HISTORY];
    uint16_t head;          /**< Index of newest point */
    uint16_t count;         /**< Number of points in list */
    uint16_t capacity;      /**< Maximum capacity */

    /* Cached calculations */
    float total_dist;       /**< Total distance of list */
    point2d_t center;       /**< Center point */
    vecteur_t delta;        /**< Extent (max-min) */

    /* Relative position tracking */
    point_t p1;             /**< Closest point */
    point_t p2;             /**< Second closest point */
    uint16_t idx_p1;        /**< Index of P1 */
    pos_relative_t pos_rel; /**< Cached relative position */
} liste_points_t;

/**
 * @brief Segment point list (static allocation for segments)
 */
typedef struct {
    point_t *points;        /**< External point array */
    uint16_t count;         /**< Number of points */
    uint16_t capacity;      /**< Array capacity */

    float total_dist;       /**< Total segment distance */
    float total_elev;       /**< Total elevation gain */
    float total_time;       /**< Total time (PR time) */
    point2d_t center;       /**< Center point */
    vecteur_t delta;        /**< Extent */

    /* Relative position */
    point_t p1;
    point_t p2;
    uint16_t idx_p1;
    pos_relative_t pos_rel;
} seg_liste_points_t;

/* ==========================================================================
 * User History Functions
 * ========================================================================== */

/**
 * @brief Initialize point list
 * @param liste Pointer to list
 * @param capacity Maximum capacity (up to LISTE_MAX_HISTORY)
 */
void liste_init(liste_points_t *liste, uint16_t capacity);

/**
 * @brief Clear all points
 * @param liste Pointer to list
 */
void liste_clear(liste_points_t *liste);

/**
 * @brief Add point to front (newest)
 * @param liste Pointer to list
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param rtime Recording time
 */
void liste_add_front(liste_points_t *liste, float lat, float lon, float alt, float rtime);

/**
 * @brief Add point to back (oldest)
 * @param liste Pointer to list
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param rtime Recording time
 */
void liste_add_back(liste_points_t *liste, float lat, float lon, float alt, float rtime);

/**
 * @brief Add point, keeping list at max size (oldest removed)
 * @param liste Pointer to list
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param rtime Recording time
 * @param max_size Maximum list size
 */
void liste_add_iso(liste_points_t *liste, float lat, float lon, float alt, float rtime, uint16_t max_size);

/**
 * @brief Get number of points
 * @param liste Pointer to list
 * @return Number of points
 */
uint16_t liste_size(const liste_points_t *liste);

/**
 * @brief Get point by index (0 = newest)
 * @param liste Pointer to list
 * @param index Index from newest
 * @return Pointer to point or NULL
 */
const point_t *liste_get_at(const liste_points_t *liste, int16_t index);

/**
 * @brief Get first (newest) point
 * @param liste Pointer to list
 * @return Pointer to point or NULL
 */
const point_t *liste_get_first(const liste_points_t *liste);

/**
 * @brief Get last (oldest) point
 * @param liste Pointer to list
 * @return Pointer to point or NULL
 */
const point_t *liste_get_last(const liste_points_t *liste);

/**
 * @brief Calculate minimum distance from point to list
 * @param liste Pointer to list
 * @param lat Latitude
 * @param lon Longitude
 * @return Minimum distance in meters
 */
float liste_distance_to(const liste_points_t *liste, float lat, float lon);

/**
 * @brief Update list statistics (center, delta, distance)
 * @param liste Pointer to list
 */
void liste_update_delta(liste_points_t *liste);

/**
 * @brief Update relative position to a point
 * @param liste Pointer to list
 * @param point Reference point
 */
void liste_update_relative_position(liste_points_t *liste, const point_t *point);

/**
 * @brief Compute relative position of a point to the list
 * @param liste Pointer to list
 * @param point Reference point
 * @return Relative position vector
 */
pos_relative_t liste_compute_pos_relative(const liste_points_t *liste, const point_t *point);

/**
 * @brief Get stored relative position
 * @param liste Pointer to list
 * @return Pointer to relative position
 */
const pos_relative_t *liste_get_pos_relative(const liste_points_t *liste);

/**
 * @brief Get P1 index (closest point)
 * @param liste Pointer to list
 * @return Index of closest point
 */
uint16_t liste_get_idx_p1(const liste_points_t *liste);

/**
 * @brief Get total elevation change
 * @param liste Pointer to list
 * @return Elevation change (last - first)
 */
float liste_get_elev_total(const liste_points_t *liste);

/**
 * @brief Get total time span
 * @param liste Pointer to list
 * @return Time span (last - first)
 */
float liste_get_time_total(const liste_points_t *liste);

/**
 * @brief Get list center point
 * @param liste Pointer to list
 * @return Pointer to center
 */
const point2d_t *liste_get_center(const liste_points_t *liste);

/**
 * @brief Get list extent
 * @param liste Pointer to list
 * @return Pointer to delta vector
 */
const vecteur_t *liste_get_delta(const liste_points_t *liste);

/* ==========================================================================
 * Segment Point List Functions
 * ========================================================================== */

/**
 * @brief Initialize segment point list with external array
 * @param liste Pointer to list
 * @param points External point array
 * @param capacity Array capacity
 */
void seg_liste_init(seg_liste_points_t *liste, point_t *points, uint16_t capacity);

/**
 * @brief Clear segment points
 * @param liste Pointer to list
 */
void seg_liste_clear(seg_liste_points_t *liste);

/**
 * @brief Add point to segment (at end)
 * @param liste Pointer to list
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param rtime Recording time
 * @return true if added, false if full
 */
bool seg_liste_add(seg_liste_points_t *liste, float lat, float lon, float alt, float rtime);

/**
 * @brief Get segment size
 * @param liste Pointer to list
 * @return Number of points
 */
uint16_t seg_liste_size(const seg_liste_points_t *liste);

/**
 * @brief Get segment point by index
 * @param liste Pointer to list
 * @param index Index (0 = first)
 * @return Pointer to point or NULL
 */
const point_t *seg_liste_get_at(const seg_liste_points_t *liste, int16_t index);

/**
 * @brief Update segment statistics
 * @param liste Pointer to list
 */
void seg_liste_update_delta(seg_liste_points_t *liste);

/**
 * @brief Update relative position to segment
 * @param liste Pointer to list
 * @param point Reference point
 */
void seg_liste_update_relative_position(seg_liste_points_t *liste, const point_t *point);

/**
 * @brief Get segment relative position
 * @param liste Pointer to list
 * @return Pointer to relative position
 */
const pos_relative_t *seg_liste_get_pos_relative(const seg_liste_points_t *liste);

/**
 * @brief Distance to P1
 * @param liste Pointer to list
 * @param lat Latitude
 * @param lon Longitude
 * @return Distance in meters
 */
float seg_liste_dist_p1(const seg_liste_points_t *liste, float lat, float lon);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_LISTE_POINTS_H_ */
