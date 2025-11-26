/**
 * @file liste_points.c
 * @brief Point list management implementation
 */

#include <string.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "model/liste_points.h"

LOG_MODULE_REGISTER(liste_points, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Distance threshold for "outside" triangle check */
#define OUTSIDE_DIST_THRESHOLD  50.0f

/** Minimum points for relative position calculation */
#define MIN_POINTS_FOR_REL_POS  5U

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Get real array index from logical index
 */
static uint16_t get_real_index(const liste_points_t *liste, int16_t logical_idx)
{
    if (liste->count == 0U) {
        return 0U;
    }

    /* Handle negative indices (-1 = last) */
    if (logical_idx < 0) {
        logical_idx = (int16_t)liste->count + logical_idx;
    }

    if ((logical_idx < 0) || ((uint16_t)logical_idx >= liste->count)) {
        return UINT16_MAX;  /* Invalid */
    }

    /* Calculate real index (head points to next write position) */
    uint16_t oldest_idx = (liste->head + liste->capacity - liste->count) % liste->capacity;
    return (oldest_idx + (uint16_t)logical_idx) % liste->capacity;
}

/**
 * @brief Find two closest points to reference
 */
static void find_closest_points(const point_t *points, uint16_t count,
                                const point_t *ref, uint16_t capacity,
                                uint16_t oldest_idx,
                                point_t *p1_out, point_t *p2_out,
                                uint16_t *idx_p1_out)
{
    point_t p1 = {0}, p2 = {0};
    float dist_p1 = 100000.0f;
    float dist_p2 = 100000.0f;
    uint16_t idx_p1 = 0U;
    uint16_t init = 0U;

    for (uint16_t i = 0U; i < count; i++) {
        uint16_t real_idx = (oldest_idx + i) % capacity;
        const point_t *pt = &points[real_idx];

        float dist = point_distance(pt, ref);

        if (init == 0U) {
            p1 = *pt;
            dist_p1 = dist;
            idx_p1 = i;
            init++;
        } else if (init == 1U) {
            p2 = *pt;
            dist_p2 = dist;
            init++;

            /* Ensure p1 is closer than p2 */
            if (dist_p1 > dist_p2) {
                point_t tmp = p1;
                float tmp_dist = dist_p1;
                p1 = p2;
                dist_p1 = dist_p2;
                p2 = tmp;
                dist_p2 = tmp_dist;
            }
        } else {
            if (dist < dist_p1) {
                p2 = p1;
                dist_p2 = dist_p1;
                p1 = *pt;
                dist_p1 = dist;
                idx_p1 = i;
            } else if (dist < dist_p2) {
                p2 = *pt;
                dist_p2 = dist;
            } else {
                /* Current point is farther than both */
            }
        }
    }

    *p1_out = p1;
    *p2_out = p2;
    *idx_p1_out = idx_p1;
}

/* ==========================================================================
 * User History Functions
 * ========================================================================== */

void liste_init(liste_points_t *liste, uint16_t capacity)
{
    if (liste == NULL) {
        return;
    }

    (void)memset(liste, 0, sizeof(liste_points_t));

    if (capacity > LISTE_MAX_HISTORY) {
        capacity = LISTE_MAX_HISTORY;
    }

    liste->capacity = capacity;
}

void liste_clear(liste_points_t *liste)
{
    if (liste == NULL) {
        return;
    }

    uint16_t cap = liste->capacity;
    (void)memset(liste, 0, sizeof(liste_points_t));
    liste->capacity = cap;
}

void liste_add_front(liste_points_t *liste, float lat, float lon, float alt, float rtime)
{
    if ((liste == NULL) || (liste->capacity == 0U)) {
        return;
    }

    /* Add at head position */
    liste->points[liste->head].lat = lat;
    liste->points[liste->head].lon = lon;
    liste->points[liste->head].alt = alt;
    liste->points[liste->head].rtime = rtime;

    /* Advance head */
    liste->head = (liste->head + 1U) % liste->capacity;

    if (liste->count < liste->capacity) {
        liste->count++;
    }
}

void liste_add_back(liste_points_t *liste, float lat, float lon, float alt, float rtime)
{
    if ((liste == NULL) || (liste->capacity == 0U)) {
        return;
    }

    if (liste->count < liste->capacity) {
        /* Calculate back position */
        uint16_t oldest = (liste->head + liste->capacity - liste->count) % liste->capacity;
        uint16_t back = (oldest + liste->capacity - 1U) % liste->capacity;

        liste->points[back].lat = lat;
        liste->points[back].lon = lon;
        liste->points[back].alt = alt;
        liste->points[back].rtime = rtime;

        liste->count++;
    }
}

void liste_add_iso(liste_points_t *liste, float lat, float lon, float alt, float rtime, uint16_t max_size)
{
    if (liste == NULL) {
        return;
    }

    /* Add to front */
    liste_add_front(liste, lat, lon, alt, rtime);

    /* Trim to max size */
    if ((max_size > 0U) && (liste->count > max_size)) {
        liste->count = max_size;
    }
}

uint16_t liste_size(const liste_points_t *liste)
{
    if (liste == NULL) {
        return 0U;
    }

    return liste->count;
}

const point_t *liste_get_at(const liste_points_t *liste, int16_t index)
{
    if ((liste == NULL) || (liste->count == 0U)) {
        return NULL;
    }

    uint16_t real_idx = get_real_index(liste, index);
    if (real_idx == UINT16_MAX) {
        return NULL;
    }

    return &liste->points[real_idx];
}

const point_t *liste_get_first(const liste_points_t *liste)
{
    return liste_get_at(liste, 0);
}

const point_t *liste_get_last(const liste_points_t *liste)
{
    return liste_get_at(liste, -1);
}

float liste_distance_to(const liste_points_t *liste, float lat, float lon)
{
    if ((liste == NULL) || (liste->count == 0U)) {
        return 100000.0f;
    }

    float min_dist = 100000.0f;

    for (uint16_t i = 0U; i < liste->count; i++) {
        const point_t *pt = liste_get_at(liste, (int16_t)i);
        if (pt != NULL) {
            float dist = distance_between(pt->lat, pt->lon, lat, lon);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
    }

    return min_dist;
}

void liste_update_delta(liste_points_t *liste)
{
    if ((liste == NULL) || (liste->count < 2U)) {
        return;
    }

    float min_lat = 200.0f, max_lat = -200.0f;
    float min_lon = 100.0f, max_lon = -100.0f;
    float total_dist = 0.0f;

    const point_t *prev = NULL;

    for (uint16_t i = 0U; i < liste->count; i++) {
        const point_t *pt = liste_get_at(liste, (int16_t)i);
        if (pt == NULL) {
            continue;
        }

        /* Track extents */
        if (pt->lat < min_lat) { min_lat = pt->lat; }
        if (pt->lat > max_lat) { max_lat = pt->lat; }
        if (pt->lon < min_lon) { min_lon = pt->lon; }
        if (pt->lon > max_lon) { max_lon = pt->lon; }

        /* Accumulate distance */
        if (prev != NULL) {
            total_dist += point_distance(prev, pt);
        }

        prev = pt;
    }

    /* Update delta */
    liste->delta.x = max_lon - min_lon;
    liste->delta.y = max_lat - min_lat;
    liste->delta.z = liste_get_elev_total(liste);
    liste->delta.t = liste_get_time_total(liste);

    /* Update center */
    liste->center.lat = 0.5f * (max_lat + min_lat);
    liste->center.lon = 0.5f * (max_lon + min_lon);

    liste->total_dist = total_dist;
}

void liste_update_relative_position(liste_points_t *liste, const point_t *point)
{
    if ((liste == NULL) || (point == NULL) || (liste->count < MIN_POINTS_FOR_REL_POS)) {
        return;
    }

    /* Find two closest points */
    uint16_t oldest_idx = (liste->head + liste->capacity - liste->count) % liste->capacity;
    find_closest_points(liste->points, liste->count, point, liste->capacity,
                        oldest_idx, &liste->p1, &liste->p2, &liste->idx_p1);

    /* Calculate relative position */
    float dist_p1 = point_distance(&liste->p1, point);
    float dist_p2 = point_distance(&liste->p2, point);
    float dist_p1p2 = point_distance(&liste->p1, &liste->p2);

    if ((dist_p1 > OUTSIDE_DIST_THRESHOLD) ||
        ((dist_p2 * dist_p2) >= ((dist_p1p2 * dist_p1p2) + (dist_p1 * dist_p1)))) {
        /* Outside triangle */
        liste->pos_rel.x = 0.0f;
        liste->pos_rel.y = 0.0f;
        liste->pos_rel.z = liste->p1.alt;
        liste->pos_rel.t = liste->p1.rtime;
    } else {
        /* Inside triangle - calculate projection */
        (void)calculate_relative_position(point, &liste->p1, &liste->p2, &liste->pos_rel);
    }
}

pos_relative_t liste_compute_pos_relative(const liste_points_t *liste, const point_t *point)
{
    pos_relative_t result = {0};

    if ((liste == NULL) || (point == NULL) || (liste->count < MIN_POINTS_FOR_REL_POS)) {
        return result;
    }

    /* Find two closest points (temporary, don't modify liste) */
    point_t p1, p2;
    uint16_t idx_p1;
    uint16_t oldest_idx = (liste->head + liste->capacity - liste->count) % liste->capacity;

    find_closest_points(liste->points, liste->count, point, liste->capacity,
                        oldest_idx, &p1, &p2, &idx_p1);

    float dist_p1 = point_distance(&p1, point);
    float dist_p2 = point_distance(&p2, point);
    float dist_p1p2 = point_distance(&p1, &p2);

    if ((dist_p1 > OUTSIDE_DIST_THRESHOLD) ||
        ((dist_p2 * dist_p2) >= ((dist_p1p2 * dist_p1p2) + (dist_p1 * dist_p1)))) {
        result.x = 0.0f;
        result.y = 0.0f;
        result.z = p1.alt;
        result.t = p1.rtime;
    } else {
        (void)calculate_relative_position(point, &p1, &p2, &result);
    }

    return result;
}

const pos_relative_t *liste_get_pos_relative(const liste_points_t *liste)
{
    if (liste == NULL) {
        return NULL;
    }

    return &liste->pos_rel;
}

uint16_t liste_get_idx_p1(const liste_points_t *liste)
{
    if (liste == NULL) {
        return 0U;
    }

    return liste->idx_p1;
}

float liste_get_elev_total(const liste_points_t *liste)
{
    const point_t *first = liste_get_first(liste);
    const point_t *last = liste_get_last(liste);

    if ((first == NULL) || (last == NULL)) {
        return 0.0f;
    }

    return last->alt - first->alt;
}

float liste_get_time_total(const liste_points_t *liste)
{
    const point_t *first = liste_get_first(liste);
    const point_t *last = liste_get_last(liste);

    if ((first == NULL) || (last == NULL)) {
        return 0.0f;
    }

    return last->rtime - first->rtime;
}

const point2d_t *liste_get_center(const liste_points_t *liste)
{
    if (liste == NULL) {
        return NULL;
    }

    return &liste->center;
}

const vecteur_t *liste_get_delta(const liste_points_t *liste)
{
    if (liste == NULL) {
        return NULL;
    }

    return &liste->delta;
}

/* ==========================================================================
 * Segment Point List Functions
 * ========================================================================== */

void seg_liste_init(seg_liste_points_t *liste, point_t *points, uint16_t capacity)
{
    if (liste == NULL) {
        return;
    }

    (void)memset(liste, 0, sizeof(seg_liste_points_t));
    liste->points = points;
    liste->capacity = capacity;
}

void seg_liste_clear(seg_liste_points_t *liste)
{
    if (liste == NULL) {
        return;
    }

    point_t *pts = liste->points;
    uint16_t cap = liste->capacity;

    (void)memset(liste, 0, sizeof(seg_liste_points_t));
    liste->points = pts;
    liste->capacity = cap;
}

bool seg_liste_add(seg_liste_points_t *liste, float lat, float lon, float alt, float rtime)
{
    if ((liste == NULL) || (liste->points == NULL)) {
        return false;
    }

    if (liste->count >= liste->capacity) {
        return false;
    }

    liste->points[liste->count].lat = lat;
    liste->points[liste->count].lon = lon;
    liste->points[liste->count].alt = alt;
    liste->points[liste->count].rtime = rtime;

    liste->count++;

    return true;
}

uint16_t seg_liste_size(const seg_liste_points_t *liste)
{
    if (liste == NULL) {
        return 0U;
    }

    return liste->count;
}

const point_t *seg_liste_get_at(const seg_liste_points_t *liste, int16_t index)
{
    if ((liste == NULL) || (liste->points == NULL) || (liste->count == 0U)) {
        return NULL;
    }

    /* Handle negative indices */
    if (index < 0) {
        index = (int16_t)liste->count + index;
    }

    if ((index < 0) || ((uint16_t)index >= liste->count)) {
        return NULL;
    }

    return &liste->points[index];
}

void seg_liste_update_delta(seg_liste_points_t *liste)
{
    if ((liste == NULL) || (liste->count < 2U)) {
        return;
    }

    float min_lat = 200.0f, max_lat = -200.0f;
    float min_lon = 100.0f, max_lon = -100.0f;
    float total_dist = 0.0f;

    const point_t *prev = NULL;

    for (uint16_t i = 0U; i < liste->count; i++) {
        const point_t *pt = seg_liste_get_at(liste, (int16_t)i);
        if (pt == NULL) {
            continue;
        }

        if (pt->lat < min_lat) { min_lat = pt->lat; }
        if (pt->lat > max_lat) { max_lat = pt->lat; }
        if (pt->lon < min_lon) { min_lon = pt->lon; }
        if (pt->lon > max_lon) { max_lon = pt->lon; }

        if (prev != NULL) {
            total_dist += point_distance(prev, pt);
        }

        prev = pt;
    }

    liste->delta.x = max_lon - min_lon;
    liste->delta.y = max_lat - min_lat;

    const point_t *first = seg_liste_get_at(liste, 0);
    const point_t *last = seg_liste_get_at(liste, -1);

    if ((first != NULL) && (last != NULL)) {
        liste->delta.z = last->alt - first->alt;
        liste->delta.t = last->rtime - first->rtime;
        liste->total_elev = liste->delta.z;
        liste->total_time = liste->delta.t;
    }

    liste->center.lat = 0.5f * (max_lat + min_lat);
    liste->center.lon = 0.5f * (max_lon + min_lon);
    liste->total_dist = total_dist;
}

void seg_liste_update_relative_position(seg_liste_points_t *liste, const point_t *point)
{
    if ((liste == NULL) || (point == NULL) ||
        (liste->points == NULL) || (liste->count < MIN_POINTS_FOR_REL_POS)) {
        return;
    }

    /* Find two closest points */
    find_closest_points(liste->points, liste->count, point, liste->count,
                        0U, &liste->p1, &liste->p2, &liste->idx_p1);

    float dist_p1 = point_distance(&liste->p1, point);
    float dist_p2 = point_distance(&liste->p2, point);
    float dist_p1p2 = point_distance(&liste->p1, &liste->p2);

    if ((dist_p1 > OUTSIDE_DIST_THRESHOLD) ||
        ((dist_p2 * dist_p2) >= ((dist_p1p2 * dist_p1p2) + (dist_p1 * dist_p1)))) {
        liste->pos_rel.x = 0.0f;
        liste->pos_rel.y = 0.0f;
        liste->pos_rel.z = liste->p1.alt;
        liste->pos_rel.t = liste->p1.rtime;
    } else {
        (void)calculate_relative_position(point, &liste->p1, &liste->p2, &liste->pos_rel);
    }
}

const pos_relative_t *seg_liste_get_pos_relative(const seg_liste_points_t *liste)
{
    if (liste == NULL) {
        return NULL;
    }

    return &liste->pos_rel;
}

float seg_liste_dist_p1(const seg_liste_points_t *liste, float lat, float lon)
{
    if (liste == NULL) {
        return 100000.0f;
    }

    return distance_between(liste->p1.lat, liste->p1.lon, lat, lon);
}
