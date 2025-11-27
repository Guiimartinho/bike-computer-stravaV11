/**
 * @file segment.c
 * @brief Strava Segment management implementation
 *
 * Based on original Segment.cpp with proper activation/deactivation
 * using vector math and Pythagorean geometry.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <string.h>
#include <stdio.h>

#include "model/segment.h"

#include "model/locator.h"
#include "model/vecteur.h"
#include "model/liste_points.h"
#include <math.h>

LOG_MODULE_REGISTER(segment, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Segment files directory */
#define SEG_DIR             "/SD:/segments"

/** Maximum filename length */
#define MAX_FILENAME_LEN    32U

/** Segment countdown after finish */
#define SEG_FINISH_COUNTDOWN    (-5)

/** Scalar product limit for activation (from original Segment.h line 25) */
#define PSCAL_LIM           0.0f

/** Margin factor for deactivation (from original Segment.h line 22) */
#define MARGE_ACT           1.5f

/** Maximum number of segment points to cache */
#define MAX_SEG_POINTS      500U

/* ==========================================================================
 * Private Types
 * ========================================================================== */

/**
 * @brief Extended segment data (runtime state)
 */
typedef struct {
    float start_time;       /**< Time when segment was activated */
    float cur_time;         /**< Current time on segment */
    float advance;          /**< Time advance/behind reference */
    float elev_start;       /**< Elevation at start */
    float elev_total;       /**< Total elevation of segment */
    float pct_dist;         /**< Progress percentage by distance */
    float pct_elev;         /**< Progress percentage by elevation */
    liste_points_t pts;     /**< Segment points list */
    bool pts_loaded;        /**< True if points are loaded */
} seg_runtime_t;

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Loaded segments */
static segment_t segments[MAX_SEGMENTS];
static uint16_t segment_count;

/** Segment headers (metadata) */
static seg_header_t seg_headers[MAX_SEGMENTS];

/** Runtime data for segments */
static seg_runtime_t seg_runtime[MAX_SEGMENTS];

/** User's GPS position history */
static liste_points_t user_history;

/** Status change callback */
static seg_status_callback_t status_callback;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Function Forward Declarations
 * ========================================================================== */

static int load_segment_points(uint16_t seg_idx);
static void unload_segment_points(uint16_t seg_idx);
static float dist_to_seg_header(uint16_t seg_idx, float lat, float lon);

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Test segment activation using vector math
 *
 * Uses scalar product of movement vector and segment direction,
 * plus Pythagorean geometry to determine if segment should activate.
 *
 * @param seg_idx Segment index
 * @return true if segment should be activated
 */
static bool test_activation(uint16_t seg_idx)
{
    seg_runtime_t *rt = &seg_runtime[seg_idx];

    /* Need at least 2 points in both lists */
    if ((rt->pts.count < 2U) || (user_history.count < 2U)) {
        return false;
    }

    /* Get segment first two points */
    const point_t *seg_p1 = liste_get_at(&rt->pts, 0);
    const point_t *seg_p2 = liste_get_at(&rt->pts, 1);
    if ((seg_p1 == NULL) || (seg_p2 == NULL)) {
        return false;
    }

    /* Get user's current and previous positions */
    const point_t *cur_pos = liste_get_at(&user_history, 0);
    const point_t *prev_pos = liste_get_at(&user_history, 1);
    if ((cur_pos == NULL) || (prev_pos == NULL)) {
        return false;
    }

    /* Test activation using vector math (from vecteur module) */
    return test_segment_activation(cur_pos, prev_pos,
                                   seg_p1, seg_p2,
                                   SEG_ACTIVATE_DIST, PSCAL_LIM);
}

/**
 * @brief Test segment deactivation using Pythagorean geometry
 *
 * Tests if user has passed the end of the segment.
 *
 * @param seg_idx Segment index
 * @return true if segment should be deactivated (finished)
 */
static bool test_deactivation(uint16_t seg_idx)
{
    seg_runtime_t *rt = &seg_runtime[seg_idx];

    /* Need points in segment */
    if (rt->pts.count < 3U) {
        return false;
    }

    /* Get user's current position */
    const point_t *cur_pos = liste_get_at(&user_history, 0);
    if (cur_pos == NULL) {
        return false;
    }

    /* Get segment last two points */
    const point_t *seg_last = liste_get_at(&rt->pts, (int16_t)(rt->pts.count - 1U));
    const point_t *seg_prev = liste_get_at(&rt->pts, (int16_t)(rt->pts.count - 2U));
    if ((seg_last == NULL) || (seg_prev == NULL)) {
        return false;
    }

    /* Test deactivation using vector math */
    return test_segment_deactivation(cur_pos, seg_prev,
                                     seg_last, SEG_ACTIVATE_DIST);
}

/**
 * @brief Calculate distance from point to segment start
 */
static float dist_to_start(uint16_t seg_idx, const loc_data_t *loc)
{
    seg_runtime_t *rt = &seg_runtime[seg_idx];

    if ((rt->pts.count == 0U) || (loc == NULL)) {
        return 9999.0f;
    }

    const point_t *seg_start = liste_get_at(&rt->pts, 0);
    if (seg_start == NULL) {
        return 9999.0f;
    }

    point_t cur_pos = {
        .lat = loc->lat,
        .lon = loc->lon,
        .alt = loc->alt,
        .rtime = 0.0f
    };

    return point_distance(&cur_pos, seg_start);
}

/**
 * @brief Update segment progress and advance
 * @param seg_idx Segment index
 * @param loc Current location
 * @param current_time Current ride time
 */
static void update_progress(uint16_t seg_idx, const loc_data_t *loc,
                           float current_time)
{
    seg_runtime_t *rt = &seg_runtime[seg_idx];
    segment_t *seg = &segments[seg_idx];

    if (rt->pts.count == 0U) {
        return;
    }

    /* Convert location to point */
    point_t cur_pos = {
        .lat = loc->lat,
        .lon = loc->lon,
        .alt = loc->alt,
        .rtime = current_time
    };

    /* Update relative position on segment */
    liste_update_relative_position(&rt->pts, &cur_pos);

    /* Get interpolated position */
    float rel_time = rt->pts.pos_rel.t;
    float rel_dist = rt->pts.pos_rel.y; /* Perpendicular distance */

    /* Check if too far from segment line */
    if (fabsf(rel_dist) > MARGE_ACT * SEG_ACTIVATE_DIST) {
        seg->status = SEG_OFF;
        LOG_INF("Segment %s deactivated (too far from line)", seg->name);
        return;
    }

    /* Calculate current time on segment */
    rt->cur_time = current_time - rt->start_time;

    /* Get reference time from segment first point */
    const point_t *first_pt = liste_get_at(&rt->pts, 0);
    if (first_pt != NULL) {
        rt->advance = (rel_time - first_pt->rtime) - rt->cur_time;
    }

    /* Update progress percentage */
    rt->pct_dist = (float)rt->pts.idx_p1 / (float)rt->pts.count;

    /* Update elevation progress */
    if (rt->elev_total > 5.0f) {
        float elev_at_pos = rt->pts.pos_rel.z;
        rt->pct_elev = (elev_at_pos - rt->elev_start) / rt->elev_total;
    }

    /* Copy to segment structure */
    seg->cur_time = rt->cur_time;
    seg->advance = rt->advance;
    seg->pct_dist = rt->pct_dist;
    seg->pct_elev = rt->pct_elev;
}

/**
 * @brief Add user position to history
 */
static void add_user_position(const loc_data_t *loc, float current_time)
{
    liste_add_iso(&user_history, loc->lat, loc->lon, loc->alt, current_time, HISTO_POINT_SIZE);
}

/**
 * @brief Update single segment state
 *
 * Implements the state machine from original Segment::majPerformance()
 */
static void update_segment(uint16_t idx, const loc_data_t *loc, float current_time)
{
    segment_t *seg = &segments[idx];
    seg_runtime_t *rt = &seg_runtime[idx];

    /* Skip if no points loaded */
    if (!rt->pts_loaded || (rt->pts.count < 3U)) {
        return;
    }

    switch (seg->status) {
    case SEG_OFF:
        /* Check if close enough to start to test activation */
        {
            float dist = dist_to_start(idx, loc);
            if (dist > SEG_ACTIVATE_DIST) {
                break; /* Too far, skip */
            }

            /* Test activation with vector math */
            if (test_activation(idx)) {
                /* Get interpolated start time from user history */
                const point_t *seg_first = liste_get_at(&rt->pts, 0);
                if (seg_first != NULL) {
                    liste_update_relative_position(&user_history, seg_first);
                    rt->start_time = user_history.pos_rel.t;
                } else {
                    rt->start_time = current_time;
                }

                rt->cur_time = 0.0f;
                rt->advance = 0.0f;
                rt->elev_start = loc->alt;
                rt->pct_dist = 0.0f;
                rt->pct_elev = 0.0f;

                seg->status = SEG_START;
                seg->cur_time = 0.0f;
                seg->advance = 0.0f;
                seg->pct_dist = 0.0f;
                seg->pct_elev = 0.0f;
                seg->score = 2; /* Starting priority */

                LOG_INF("Segment %s activated", seg->name);

                if (status_callback != NULL) {
                    status_callback(seg);
                }
            }
        }
        break;

    case SEG_START:
        /* Transition to active */
        seg->status = SEG_ON;
        /* Fall through */
        __attribute__((fallthrough));

    case SEG_ON:
        /* Test for segment finish (deactivation) */
        if (test_deactivation(idx)) {
            /* Segment finished - calculate final advance */
            const point_t *seg_last = liste_get_at(&rt->pts, (int16_t)(rt->pts.count - 1U));
            if (seg_last != NULL) {
                liste_update_relative_position(&user_history, seg_last);
                rt->cur_time = user_history.pos_rel.t - rt->start_time;
                rt->advance = seg->total_time - rt->cur_time;
            }

            rt->pct_dist = 1.0f;
            seg->cur_time = rt->cur_time;
            seg->advance = rt->advance;
            seg->pct_dist = 1.0f;
            seg->status = SEG_FIN;
            seg->score = 10; /* Highest priority for finished segment */

            LOG_INF("Segment %s finished! Time: %.1f s, Advance: %.1f s",
                    seg->name, (double)seg->cur_time, (double)seg->advance);

            if (status_callback != NULL) {
                status_callback(seg);
            }
        } else {
            /* Update progress */
            update_progress(idx, loc, current_time);

            /* Update score based on progress (2 to 9) */
            seg->score = 2 + (int8_t)(rt->pct_dist * 7.0f);
        }
        break;

    case SEG_FIN:
        /* Countdown to removal from display */
        seg->score--;
        if (seg->score < SEG_FINISH_COUNTDOWN) {
            seg->status = SEG_OFF;
            seg->score = 0;
        }
        break;

    default:
        seg->status = SEG_OFF;
        break;
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t segment_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Clear segments */
    (void)memset(segments, 0, sizeof(segments));
    (void)memset(seg_headers, 0, sizeof(seg_headers));
    (void)memset(seg_runtime, 0, sizeof(seg_runtime));
    segment_count = 0U;

    /* Initialize user position history */
    liste_init(&user_history, LISTE_MAX_HISTORY);

    is_initialized = true;
    LOG_INF("Segment manager initialized");

    return APP_OK;
}

int segment_load_all(void)
{
    if (!is_initialized) {
        return (int)APP_ERR_NOT_INIT;
    }

    struct fs_dir_t dir;
    struct fs_dirent entry;
    int count = 0;

    fs_dir_t_init(&dir);

    int err = fs_opendir(&dir, SEG_DIR);
    if (err < 0) {
        LOG_WRN("Cannot open segments directory: %d", err);
        return 0;
    }

    while ((fs_readdir(&dir, &entry) == 0) && (entry.name[0] != '\0')) {
        if (entry.type != FS_DIR_ENTRY_FILE) {
            continue;
        }

        /* Check for .seg extension */
        size_t len = strlen(entry.name);
        if ((len < 5U) || (strcmp(&entry.name[len - 4], ".seg") != 0)) {
            continue;
        }

        if (segment_count >= MAX_SEGMENTS) {
            LOG_WRN("Maximum segments reached");
            break;
        }

        /* Load segment header */
        char path[280];  /* SEG_DIR + "/" + max filename (255) */
        (void)snprintf(path, sizeof(path), "%s/%s", SEG_DIR, entry.name);

        struct fs_file_t file;
        fs_file_t_init(&file);

        if (fs_open(&file, path, FS_O_READ) == 0) {
            /* Read header */
            ssize_t bytes = fs_read(&file, &seg_headers[segment_count],
                                    sizeof(seg_header_t));

            if (bytes == sizeof(seg_header_t)) {
                /* Initialize segment state */
                segment_t *seg = &segments[segment_count];
                (void)strncpy(seg->name, seg_headers[segment_count].name,
                             sizeof(seg->name) - 1U);
                seg->num_points = seg_headers[segment_count].num_points;
                seg->total_time = seg_headers[segment_count].total_time;
                seg->total_elev = seg_headers[segment_count].total_elev;
                seg->status = SEG_OFF;
                seg->score = 0;

                segment_count++;
                count++;

                LOG_INF("Loaded segment: %s (%u points)",
                        seg->name, seg->num_points);
            }

            (void)fs_close(&file);
        }
    }

    (void)fs_closedir(&dir);

    LOG_INF("Loaded %d segments", count);
    return count;
}

app_err_t segment_update(const loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (loc == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Get current time from location timestamp */
    float current_time = (float)loc->timestamp / 1000.0f;

    /* Add position to user history */
    add_user_position(loc, current_time);

    /* Update all segments */
    for (uint16_t i = 0U; i < segment_count; i++) {
        update_segment(i, loc, current_time);
    }

    return APP_OK;
}

uint8_t segment_get_active_count(void)
{
    uint8_t count = 0U;

    for (uint16_t i = 0U; i < segment_count; i++) {
        if ((segments[i].status == SEG_START) ||
            (segments[i].status == SEG_ON) ||
            (segments[i].status == SEG_FIN)) {
            count++;
        }
    }

    return count;
}

app_err_t segment_get(uint8_t index, segment_t *seg)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((index >= segment_count) || (seg == NULL)) {
        return APP_ERR_INVALID_PARAM;
    }

    *seg = segments[index];
    return APP_OK;
}

app_err_t segment_get_best(segment_t *seg)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (seg == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Find active segment with highest score */
    segment_t *best = NULL;
    int8_t best_score = INT8_MIN;

    for (uint16_t i = 0U; i < segment_count; i++) {
        if ((segments[i].status == SEG_ON) ||
            (segments[i].status == SEG_START)) {
            if (segments[i].score > best_score) {
                best = &segments[i];
                best_score = segments[i].score;
            }
        }
    }

    if (best == NULL) {
        return APP_ERR_NOT_FOUND;
    }

    *seg = *best;
    return APP_OK;
}

uint8_t segment_get_active(segment_t *segs, uint8_t max_count)
{
    if (!is_initialized || (segs == NULL) || (max_count == 0U)) {
        return 0U;
    }

    uint8_t count = 0U;

    /* Collect active segments */
    for (uint16_t i = 0U; (i < segment_count) && (count < max_count); i++) {
        if ((segments[i].status == SEG_START) ||
            (segments[i].status == SEG_ON) ||
            (segments[i].status == SEG_FIN)) {
            segs[count] = segments[i];
            count++;
        }
    }

    /* Sort by score (descending) using insertion sort - efficient for small arrays */
    for (uint8_t i = 1U; i < count; i++) {
        segment_t temp = segs[i];
        int8_t j = (int8_t)i - 1;

        while ((j >= 0) && (segs[j].score < temp.score)) {
            segs[j + 1] = segs[j];
            j--;
        }
        segs[j + 1] = temp;
    }

    return count;
}

uint8_t segment_get_nearby(segment_t *segs, uint8_t max_count, float lat, float lon)
{
    if (!is_initialized || (segs == NULL) || (max_count == 0U)) {
        return 0U;
    }

    /* Temporary array to store segment indices and distances */
    typedef struct {
        uint16_t idx;
        float dist;
    } seg_dist_t;

    seg_dist_t seg_dists[MAX_SEGMENTS];
    uint8_t count = 0U;

    /* Calculate distance for all loaded segments */
    for (uint16_t i = 0U; i < segment_count; i++) {
        seg_runtime_t *rt = &seg_runtime[i];

        /* Only include segments with loaded points */
        if (!rt->pts_loaded || (rt->pts.count == 0U)) {
            continue;
        }

        /* Get distance to first point */
        const point_t *seg_start = liste_get_at(&rt->pts, 0);
        if (seg_start == NULL) {
            continue;
        }

        point_t cur = { .lat = lat, .lon = lon, .alt = 0.0f, .rtime = 0.0f };
        float dist = point_distance(&cur, seg_start);

        seg_dists[count].idx = i;
        seg_dists[count].dist = dist;
        count++;

        if (count >= MAX_SEGMENTS) {
            break;
        }
    }

    /* Sort by distance (ascending) using insertion sort */
    for (uint8_t i = 1U; i < count; i++) {
        seg_dist_t temp = seg_dists[i];
        int8_t j = (int8_t)i - 1;

        while ((j >= 0) && (seg_dists[j].dist > temp.dist)) {
            seg_dists[j + 1] = seg_dists[j];
            j--;
        }
        seg_dists[j + 1] = temp;
    }

    /* Copy sorted segments to output array */
    uint8_t result_count = (count < max_count) ? count : max_count;
    for (uint8_t i = 0U; i < result_count; i++) {
        segs[i] = segments[seg_dists[i].idx];
    }

    return result_count;
}

app_err_t segment_register_callback(seg_status_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    status_callback = callback;
    return APP_OK;
}

float segment_get_nearest_distance(void)
{
    if (!is_initialized || (segment_count == 0U)) {
        return -1.0f;
    }

    float nearest = 9999.0f;

    for (uint16_t i = 0U; i < segment_count; i++) {
        seg_runtime_t *rt = &seg_runtime[i];

        /* Skip if points not loaded */
        if (!rt->pts_loaded || (rt->pts.count == 0U)) {
            continue;
        }

        /* Get distance to first point of segment */
        const point_t *seg_start = liste_get_at(&rt->pts, 0);
        if (seg_start == NULL) {
            continue;
        }

        /* Get current position from user history */
        const point_t *cur_pos = liste_get_at(&user_history, 0);
        if (cur_pos == NULL) {
            continue;
        }

        float dist = point_distance(cur_pos, seg_start);
        if (dist < nearest) {
            nearest = dist;
        }
    }

    return (nearest < 9000.0f) ? nearest : -1.0f;
}

uint16_t segment_get_total_count(void)
{
    return segment_count;
}

bool segment_is_any_active(void)
{
    return (segment_get_active_count() > 0U);
}

void segment_reset_all(void)
{
    for (uint16_t i = 0U; i < segment_count; i++) {
        segments[i].status = SEG_OFF;
        segments[i].cur_time = 0.0f;
        segments[i].advance = 0.0f;
        segments[i].pct_dist = 0.0f;
        segments[i].pct_elev = 0.0f;
        segments[i].score = 0;

        /* Reset runtime state but keep points loaded */
        seg_runtime[i].start_time = 0.0f;
        seg_runtime[i].cur_time = 0.0f;
        seg_runtime[i].advance = 0.0f;
        seg_runtime[i].pct_dist = 0.0f;
        seg_runtime[i].pct_elev = 0.0f;
    }

    /* Clear user position history */
    liste_clear(&user_history);

    LOG_INF("All segments reset");
}

void segment_unload_all(void)
{
    /* Clear runtime data including point lists */
    for (uint16_t i = 0U; i < segment_count; i++) {
        liste_clear(&seg_runtime[i].pts);
    }

    (void)memset(segments, 0, sizeof(segments));
    (void)memset(seg_headers, 0, sizeof(seg_headers));
    (void)memset(seg_runtime, 0, sizeof(seg_runtime));
    segment_count = 0U;

    /* Clear user position history */
    liste_clear(&user_history);

    LOG_INF("All segments unloaded");
}

/**
 * @brief Load segment points from file
 */
static int load_segment_points(uint16_t seg_idx)
{
    if (seg_idx >= segment_count) {
        return -1;
    }

    seg_runtime_t *rt = &seg_runtime[seg_idx];
    segment_t *seg = &segments[seg_idx];

    if (rt->pts_loaded) {
        return 0;  /* Already loaded */
    }

    /* Build filename */
    char path[64];
    (void)snprintf(path, sizeof(path), "%s/%s.seg", SEG_DIR, seg->name);

    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, path, FS_O_READ) < 0) {
        LOG_WRN("Cannot open segment file: %s", path);
        return -1;
    }

    /* Skip header */
    (void)fs_seek(&file, (off_t)sizeof(seg_header_t), FS_SEEK_SET);

    /* Initialize point list */
    liste_init(&rt->pts, MAX_SEG_POINTS);

    /* Read points */
    seg_point_t pt;
    int count = 0;
    while (fs_read(&file, &pt, sizeof(seg_point_t)) == sizeof(seg_point_t)) {
        liste_add_back(&rt->pts, pt.lat, pt.lon, pt.alt, pt.time);
        count++;

        if ((uint16_t)count >= MAX_SEG_POINTS) {
            LOG_WRN("Segment %s truncated at %d points", seg->name, count);
            break;
        }
    }

    (void)fs_close(&file);

    if (count > 0) {
        rt->pts_loaded = true;
        rt->elev_total = seg->total_elev;

        LOG_INF("Loaded %d points for segment %s", count, seg->name);
        return count;
    }

    return -1;
}

/**
 * @brief Unload segment points to free memory
 */
static void unload_segment_points(uint16_t seg_idx)
{
    if (seg_idx >= segment_count) {
        return;
    }

    seg_runtime_t *rt = &seg_runtime[seg_idx];
    segment_t *seg = &segments[seg_idx];

    if (!rt->pts_loaded) {
        return;
    }

    liste_clear(&rt->pts);
    rt->pts_loaded = false;

    /* Reset segment state */
    seg->status = SEG_OFF;
    seg->cur_time = 0.0f;
    seg->advance = 0.0f;
    seg->pct_dist = 0.0f;
    seg->score = 0;

    LOG_INF("Unloaded segment %s", seg->name);
}

/**
 * @brief Calculate distance from coordinates to segment start using header
 */
static float dist_to_seg_header(uint16_t seg_idx, float lat, float lon)
{
    if (seg_idx >= segment_count) {
        return 9999.0f;
    }

    /* Parse segment name for coordinates (format: LLLLL#LLL.seg)
     * Where LLLLL is lat*100 and LLL is lon*100 */
    seg_header_t *hdr = &seg_headers[seg_idx];

    /* For now use a simplified approach - load first point if needed */
    seg_runtime_t *rt = &seg_runtime[seg_idx];

    if (rt->pts_loaded && (rt->pts.count > 0U)) {
        const point_t *seg_start = liste_get_at(&rt->pts, 0);
        if (seg_start != NULL) {
            point_t cur = { .lat = lat, .lon = lon, .alt = 0.0f, .rtime = 0.0f };
            return point_distance(&cur, seg_start);
        }
    }

    /* If points not loaded, estimate from header name parsing */
    /* This is a simplified implementation - full implementation would
     * parse the segment filename for coordinates */
    (void)hdr;  /* Avoid unused warning */

    return 9999.0f;
}

float segment_allocator(uint16_t seg_idx, float lat, float lon)
{
    if (!is_initialized || (seg_idx >= segment_count)) {
        return -1.0f;
    }

    seg_runtime_t *rt = &seg_runtime[seg_idx];
    segment_t *seg = &segments[seg_idx];
    float dist_to_seg = 9999.0f;

    /* Segment is loaded with points */
    if (rt->pts_loaded && (rt->pts.count > 0U)) {
        /* Get distance to first point */
        const point_t *seg_start = liste_get_at(&rt->pts, 0);
        if (seg_start != NULL) {
            point_t cur = { .lat = lat, .lon = lon, .alt = 0.0f, .rtime = 0.0f };
            dist_to_seg = point_distance(&cur, seg_start);
        }

        /* Check if segment is inactive and too far - unload */
        if ((seg->status == SEG_OFF) && (dist_to_seg > SEG_ALLOC_DIST)) {
            unload_segment_points(seg_idx);
            LOG_DBG("Unallocated segment %s (dist=%.0f)", seg->name, (double)dist_to_seg);
        }
        /* Check if active but way too far - force unload */
        else if (dist_to_seg > SEG_MARGE_DESACT * SEG_ALLOC_DIST) {
            unload_segment_points(seg_idx);
            LOG_WRN("Force unallocated segment %s", seg->name);
        }
    }
    /* Segment not loaded - check if should load */
    else {
        dist_to_seg = dist_to_seg_header(seg_idx, lat, lon);

        if (dist_to_seg < SEG_ALLOC_DIST) {
            int res = load_segment_points(seg_idx);
            if (res > 0) {
                LOG_INF("Allocated segment %s (dist=%.0f)", seg->name, (double)dist_to_seg);
            }
        }
    }

    return dist_to_seg;
}

float segment_run_allocator(float lat, float lon)
{
    if (!is_initialized || (segment_count == 0U)) {
        return -1.0f;
    }

    float nearest = 9999.0f;

    for (uint16_t i = 0U; i < segment_count; i++) {
        float dist = segment_allocator(i, lat, lon);
        if ((dist >= 0.0f) && (dist < nearest)) {
            nearest = dist;
        }
    }

    return (nearest < 9000.0f) ? nearest : -1.0f;
}
