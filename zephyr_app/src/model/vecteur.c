/**
 * @file vecteur.c
 * @brief 2D/3D Vector operations implementation
 */

#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include "model/vecteur.h"

LOG_MODULE_REGISTER(vecteur, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Earth radius in meters */
#define EARTH_RADIUS_M      6371000.0f

/** Minimum norm to avoid division by zero */
#define MIN_NORM            0.001f

/** Degrees to radians conversion */
#define DEG_TO_RAD(deg)     ((deg) * 0.017453292519943295f)

/* ==========================================================================
 * Point Functions
 * ========================================================================== */

void point_init(point_t *pt, float lat, float lon, float alt, float rtime)
{
    if (pt == NULL) {
        return;
    }

    pt->lat = lat;
    pt->lon = lon;
    pt->alt = alt;
    pt->rtime = rtime;
}

void point2d_init(point2d_t *pt, float lat, float lon)
{
    if (pt == NULL) {
        return;
    }

    pt->lat = lat;
    pt->lon = lon;
}

bool point_is_valid(const point_t *pt)
{
    if (pt == NULL) {
        return false;
    }

    /* Check latitude range */
    if ((pt->lat == 0.0f) || (fabsf(pt->lat) > 89.0f)) {
        return false;
    }

    /* Check longitude range */
    if ((pt->lon == 0.0f) || (fabsf(pt->lon) > 189.0f)) {
        return false;
    }

    return true;
}

float distance_between(float lat1, float lon1, float lat2, float lon2)
{
    /* Haversine formula */
    float dlat = DEG_TO_RAD(lat2 - lat1);
    float dlon = DEG_TO_RAD(lon2 - lon1);

    float lat1_rad = DEG_TO_RAD(lat1);
    float lat2_rad = DEG_TO_RAD(lat2);

    float a = sinf(dlat / 2.0f) * sinf(dlat / 2.0f) +
              cosf(lat1_rad) * cosf(lat2_rad) *
              sinf(dlon / 2.0f) * sinf(dlon / 2.0f);

    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    return EARTH_RADIUS_M * c;
}

float point_distance(const point_t *p1, const point_t *p2)
{
    if ((p1 == NULL) || (p2 == NULL)) {
        return 0.0f;
    }

    return distance_between(p1->lat, p1->lon, p2->lat, p2->lon);
}

float point2d_distance(const point2d_t *p1, const point2d_t *p2)
{
    if ((p1 == NULL) || (p2 == NULL)) {
        return 0.0f;
    }

    return distance_between(p1->lat, p1->lon, p2->lat, p2->lon);
}

/* ==========================================================================
 * Vector Functions
 * ========================================================================== */

void vecteur_init(vecteur_t *v, float x, float y, float z, float t)
{
    if (v == NULL) {
        return;
    }

    v->x = x;
    v->y = y;
    v->z = z;
    v->t = t;
}

void vecteur_from_points(vecteur_t *v, const point_t *p1, const point_t *p2)
{
    if ((v == NULL) || (p1 == NULL) || (p2 == NULL)) {
        return;
    }

    /* Calculate X (east-west) distance */
    v->x = distance_between(p1->lat, p1->lon, p1->lat, p2->lon);

    /* Calculate Y (north-south) distance */
    v->y = distance_between(p1->lat, p1->lon, p2->lat, p1->lon);

    /* Apply sign based on direction */
    if (p2->lat < p1->lat) {
        v->y = -v->y;
    }
    if (p2->lon < p1->lon) {
        v->x = -v->x;
    }

    v->z = p2->alt - p1->alt;
    v->t = p2->rtime - p1->rtime;
}

float vecteur_get_norm(const vecteur_t *v)
{
    if (v == NULL) {
        return 0.0f;
    }

    float norm_sq = (v->x * v->x) + (v->y * v->y);
    return sqrtf(norm_sq);
}

void vecteur_normalize(vecteur_t *v)
{
    if (v == NULL) {
        return;
    }

    float norm = vecteur_get_norm(v);

    if (norm < MIN_NORM) {
        return;
    }

    v->x /= norm;
    v->y /= norm;
}

float vecteur_scalar_product(const vecteur_t *v1, const vecteur_t *v2)
{
    if ((v1 == NULL) || (v2 == NULL)) {
        return 0.0f;
    }

    return (v1->x * v2->x) + (v1->y * v2->y);
}

void vecteur_project(const vecteur_t *v1, const vecteur_t *v2, vecteur_t *result)
{
    if ((v1 == NULL) || (v2 == NULL) || (result == NULL)) {
        return;
    }

    float norm_sq = (v2->x * v2->x) + (v2->y * v2->y);

    if (norm_sq < (MIN_NORM * MIN_NORM)) {
        vecteur_init(result, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    float dot = vecteur_scalar_product(v1, v2);
    float scale = dot / norm_sq;

    result->x = scale * v2->x;
    result->y = scale * v2->y;
    result->z = 0.0f;
    result->t = 0.0f;
}

void vecteur_orthogonal(const vecteur_t *v, vecteur_t *result)
{
    if ((v == NULL) || (result == NULL)) {
        return;
    }

    /* Rotate 90 degrees: (x, y) -> (y, -x) */
    result->x = v->y;
    result->y = -v->x;
    result->z = 0.0f;
    result->t = 0.0f;
}

/* ==========================================================================
 * Segment Geometry Functions
 * ========================================================================== */

bool calculate_relative_position(const point_t *point,
                                 const point_t *seg_p1,
                                 const point_t *seg_p2,
                                 pos_relative_t *result)
{
    if ((point == NULL) || (seg_p1 == NULL) || (seg_p2 == NULL) || (result == NULL)) {
        return false;
    }

    /* Initialize result */
    result->x = 0.0f;
    result->y = 0.0f;
    result->z = seg_p1->alt;
    result->t = seg_p1->rtime;

    float dist_p1 = point_distance(seg_p1, point);
    float dist_p2 = point_distance(seg_p2, point);
    float dist_p1p2 = point_distance(seg_p1, seg_p2);

    /* Check if outside segment projection (Pythagorean check) */
    if ((dist_p1 > 50.0f) ||
        ((dist_p2 * dist_p2) >= ((dist_p1p2 * dist_p1p2) + (dist_p1 * dist_p1)))) {
        /* Outside triangle - return P1 position */
        return false;
    }

    /* Inside triangle - calculate projection */
    vecteur_t p1_to_point;
    vecteur_t p1_to_p2;

    vecteur_from_points(&p1_to_point, seg_p1, point);
    vecteur_from_points(&p1_to_p2, seg_p1, seg_p2);

    float p1p2_norm = vecteur_get_norm(&p1_to_p2);

    if (p1p2_norm < MIN_NORM) {
        return false;
    }

    /* Project point onto segment line (distance along segment) */
    float dot_product = vecteur_scalar_product(&p1_to_point, &p1_to_p2);
    result->x = dot_product / p1p2_norm;

    /* Calculate perpendicular distance */
    vecteur_t ortho;
    vecteur_orthogonal(&p1_to_p2, &ortho);

    float ortho_norm = vecteur_get_norm(&ortho);
    if (ortho_norm > MIN_NORM) {
        result->y = vecteur_scalar_product(&p1_to_point, &ortho) / ortho_norm;
    }

    /* Interpolate altitude */
    float ratio = result->x / p1p2_norm;
    result->z = seg_p1->alt + (seg_p2->alt - seg_p1->alt) * ratio;

    /* Interpolate time */
    result->t = seg_p1->rtime + (seg_p2->rtime - seg_p1->rtime) * ratio;

    return true;
}

bool test_segment_activation(const point_t *cur_pos,
                            const point_t *prev_pos,
                            const point_t *seg_p1,
                            const point_t *seg_p2,
                            float dist_threshold,
                            float pscal_limit)
{
    if ((cur_pos == NULL) || (prev_pos == NULL) ||
        (seg_p1 == NULL) || (seg_p2 == NULL)) {
        return false;
    }

    /* Check distance to first segment point */
    float dist_to_start = point_distance(seg_p1, cur_pos);

    if (dist_to_start > dist_threshold) {
        return false;
    }

    /* Create direction vectors */
    vecteur_t movement;    /* User's movement direction */
    vecteur_t seg_dir;     /* Segment direction */

    vecteur_from_points(&movement, prev_pos, cur_pos);
    vecteur_from_points(&seg_dir, seg_p1, seg_p2);

    /* Check if vectors are valid */
    if ((vecteur_get_norm(&movement) < MIN_NORM) ||
        (vecteur_get_norm(&seg_dir) < MIN_NORM)) {
        return false;
    }

    /* Normalize vectors */
    vecteur_normalize(&movement);
    vecteur_normalize(&seg_dir);

    /* Calculate scalar product (direction match) */
    float p_scal = vecteur_scalar_product(&movement, &seg_dir);

    /* Check geometry (Pythagorean) */
    float dist_p1 = point_distance(seg_p1, cur_pos);
    float dist_p2 = point_distance(seg_p2, cur_pos);
    float dist_p1p2 = point_distance(seg_p1, seg_p2);

    /* Activation conditions:
     * 1. Scalar product > limit (moving in same direction as segment)
     * 2. Geometry check: P2^2 < P1^2 + P1P2^2 (approaching P2 from P1)
     */
    if ((p_scal > pscal_limit) &&
        ((dist_p2 * dist_p2) < ((dist_p1 * dist_p1) + (dist_p1p2 * dist_p1p2)))) {
        LOG_DBG("Segment activation: pscal=%.2f, dist=%.1fm",
                (double)p_scal, (double)dist_to_start);
        return true;
    }

    return false;
}

bool test_segment_deactivation(const point_t *cur_pos,
                               const point_t *seg_p_last,
                               const point_t *seg_p_end,
                               float dist_threshold)
{
    if ((cur_pos == NULL) || (seg_p_last == NULL) || (seg_p_end == NULL)) {
        return false;
    }

    /* Calculate distances */
    float dist_p1 = point_distance(seg_p_last, cur_pos);
    float dist_p2 = point_distance(seg_p_end, cur_pos);
    float dist_p1p2 = point_distance(seg_p_last, seg_p_end);

    /* Quick check - if too far from end, not yet finished */
    if (dist_p2 > dist_threshold) {
        return false;
    }

    /* Deactivation condition (Pythagorean):
     * P1^2 > P2^2 + P1P2^2 (passed P2, end of segment)
     * AND within threshold distance
     */
    if (((dist_p1 * dist_p1) > ((dist_p2 * dist_p2) + (dist_p1p2 * dist_p1p2))) &&
        ((dist_p1 < dist_threshold) || (dist_p2 < dist_threshold))) {
        LOG_DBG("Segment deactivation: passed end point");
        return true;
    }

    return false;
}
