/**
 * @file vecteur.h
 * @brief 2D/3D Vector operations for segment calculations
 * @note Follows MISRA C:2012 guidelines
 */

#ifndef MODEL_VECTEUR_H_
#define MODEL_VECTEUR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief 2D Point structure
 */
typedef struct {
    float lat;      /**< Latitude in degrees */
    float lon;      /**< Longitude in degrees */
} point2d_t;

/**
 * @brief Full Point structure with altitude and time
 */
typedef struct {
    float lat;      /**< Latitude in degrees */
    float lon;      /**< Longitude in degrees */
    float alt;      /**< Altitude in meters */
    float rtime;    /**< Recording time in seconds */
} point_t;

/**
 * @brief Vector structure for calculations
 */
typedef struct {
    float x;        /**< X component (east-west distance) */
    float y;        /**< Y component (north-south distance) */
    float z;        /**< Z component (altitude) */
    float t;        /**< T component (time) */
} vecteur_t;

/**
 * @brief Relative position result
 */
typedef struct {
    float x;        /**< Distance along segment (meters) */
    float y;        /**< Distance perpendicular to segment (meters) */
    float z;        /**< Interpolated altitude (meters) */
    float t;        /**< Interpolated time (seconds) */
} pos_relative_t;

/* ==========================================================================
 * Point Functions
 * ========================================================================== */

/**
 * @brief Initialize point with values
 * @param pt Pointer to point
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude
 * @param rtime Recording time
 */
void point_init(point_t *pt, float lat, float lon, float alt, float rtime);

/**
 * @brief Initialize 2D point
 * @param pt Pointer to point
 * @param lat Latitude
 * @param lon Longitude
 */
void point2d_init(point2d_t *pt, float lat, float lon);

/**
 * @brief Check if point is valid
 * @param pt Pointer to point
 * @return true if valid coordinates
 */
bool point_is_valid(const point_t *pt);

/**
 * @brief Calculate distance between two points (Haversine)
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in meters
 */
float point_distance(const point_t *p1, const point_t *p2);

/**
 * @brief Calculate distance between two 2D points
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in meters
 */
float point2d_distance(const point2d_t *p1, const point2d_t *p2);

/**
 * @brief Calculate distance from lat/lon
 * @param lat1 First latitude
 * @param lon1 First longitude
 * @param lat2 Second latitude
 * @param lon2 Second longitude
 * @return Distance in meters
 */
float distance_between(float lat1, float lon1, float lat2, float lon2);

/* ==========================================================================
 * Vector Functions
 * ========================================================================== */

/**
 * @brief Initialize vector with values
 * @param v Pointer to vector
 * @param x X component
 * @param y Y component
 * @param z Z component
 * @param t T component
 */
void vecteur_init(vecteur_t *v, float x, float y, float z, float t);

/**
 * @brief Create vector from two points (P1 -> P2)
 * @param v Pointer to result vector
 * @param p1 Start point
 * @param p2 End point
 */
void vecteur_from_points(vecteur_t *v, const point_t *p1, const point_t *p2);

/**
 * @brief Get vector norm (magnitude)
 * @param v Pointer to vector
 * @return Magnitude (sqrt(x^2 + y^2))
 */
float vecteur_get_norm(const vecteur_t *v);

/**
 * @brief Normalize vector to unit length
 * @param v Pointer to vector (modified in place)
 */
void vecteur_normalize(vecteur_t *v);

/**
 * @brief Calculate scalar (dot) product of two vectors
 * @param v1 First vector
 * @param v2 Second vector
 * @return Scalar product (v1.x*v2.x + v1.y*v2.y)
 */
float vecteur_scalar_product(const vecteur_t *v1, const vecteur_t *v2);

/**
 * @brief Project vector v1 onto v2
 * @param v1 Vector to project
 * @param v2 Direction vector
 * @param result Result of projection
 */
void vecteur_project(const vecteur_t *v1, const vecteur_t *v2, vecteur_t *result);

/**
 * @brief Get orthogonal (perpendicular) vector
 * @param v Input vector
 * @param result Orthogonal vector (rotated 90 degrees)
 */
void vecteur_orthogonal(const vecteur_t *v, vecteur_t *result);

/* ==========================================================================
 * Segment Geometry Functions
 * ========================================================================== */

/**
 * @brief Calculate relative position of point to line segment
 * @param point Current position
 * @param seg_p1 Segment start point
 * @param seg_p2 Segment end point
 * @param result Relative position result
 * @return true if point is within segment projection
 */
bool calculate_relative_position(const point_t *point,
                                 const point_t *seg_p1,
                                 const point_t *seg_p2,
                                 pos_relative_t *result);

/**
 * @brief Test segment activation condition
 * @param cur_pos Current position
 * @param prev_pos Previous position
 * @param seg_p1 Segment first point
 * @param seg_p2 Segment second point
 * @param dist_threshold Distance threshold for activation
 * @param pscal_limit Scalar product limit (direction match)
 * @return true if segment should be activated
 */
bool test_segment_activation(const point_t *cur_pos,
                            const point_t *prev_pos,
                            const point_t *seg_p1,
                            const point_t *seg_p2,
                            float dist_threshold,
                            float pscal_limit);

/**
 * @brief Test segment deactivation condition
 * @param cur_pos Current position
 * @param seg_p_last Segment second to last point
 * @param seg_p_end Segment last point
 * @param dist_threshold Distance threshold
 * @return true if segment should be deactivated (finished)
 */
bool test_segment_deactivation(const point_t *cur_pos,
                               const point_t *seg_p_last,
                               const point_t *seg_p_end,
                               float dist_threshold);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_VECTEUR_H_ */
