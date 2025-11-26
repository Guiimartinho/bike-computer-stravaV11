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
#include <math.h>

#include "model/segment.h"
#include "model/locator.h"
#include "model/vecteur.h"
#include "model/liste_points.h"

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
    point_t seg_p1, seg_p2;
    if (!liste_get_point(&rt->pts, 0, &seg_p1) ||
        !liste_get_point(&rt->pts, 1, &seg_p2)) {
        return false;
    }

    /* Get user's current and previous positions */
    point_t cur_pos, prev_pos;
    if (!liste_get_point(&user_history, 0, &cur_pos) ||
        !liste_get_point(&user_history, 1, &prev_pos)) {
        return false;
    }

    /* Test activation using vector math (from vecteur module) */
    return test_segment_activation(&cur_pos, &prev_pos,
                                   &seg_p1, &seg_p2,
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
    point_t cur_pos;
    if (!liste_get_point(&user_history, 0, &cur_pos)) {
        return false;
    }

    /* Get segment last two points */
    point_t seg_last, seg_prev;
    if (!liste_get_point(&rt->pts, (int32_t)rt->pts.count - 1, &seg_last) ||
        !liste_get_point(&rt->pts, (int32_t)rt->pts.count - 2, &seg_prev)) {
        return false;
    }

    /* Test deactivation using vector math */
    return test_segment_deactivation(&cur_pos, &seg_prev,
                                     &seg_last, SEG_ACTIVATE_DIST);
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

    point_t seg_start;
    if (!liste_get_point(&rt->pts, 0, &seg_start)) {
        return 9999.0f;
    }

    point_t cur_pos = {
        .lat = loc->lat,
        .lon = loc->lon,
        .alt = loc->alt,
        .time = 0.0f
    };

    return point_distance(&cur_pos, &seg_start);
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
        .time = current_time
    };

    /* Update relative position on segment */
    liste_update_relative_position(&rt->pts, &cur_pos);

    /* Get interpolated position */
    float rel_time = rt->pts.pos_rel.time;
    float rel_dist = rt->pts.pos_rel.lat; /* Using lat as perpendicular distance */

    /* Check if too far from segment line */
    if (fabsf(rel_dist) > MARGE_ACT * SEG_ACTIVATE_DIST) {
        seg->status = SEG_OFF;
        LOG_INF("Segment %s deactivated (too far from line)", seg->name);
        return;
    }

    /* Calculate current time on segment */
    rt->cur_time = current_time - rt->start_time;

    /* Get reference time from segment first point */
    point_t first_pt;
    if (liste_get_point(&rt->pts, 0, &first_pt)) {
        rt->advance = (rel_time - first_pt.time) - rt->cur_time;
    }

    /* Update progress percentage */
    rt->pct_dist = (float)rt->pts.ind_p1 / (float)rt->pts.count;

    /* Update elevation progress */
    if (rt->elev_total > 5.0f) {
        float elev_at_pos = rt->pts.pos_rel.alt;
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
    point_t pt = {
        .lat = loc->lat,
        .lon = loc->lon,
        .alt = loc->alt,
        .time = current_time
    };

    liste_add_point(&user_history, &pt, HISTO_POINT_SIZE);
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
                point_t seg_first;
                if (liste_get_point(&rt->pts, 0, &seg_first)) {
                    liste_update_relative_position(&user_history, &seg_first);
                    rt->start_time = user_history.pos_rel.time;
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
            point_t seg_last;
            if (liste_get_point(&rt->pts, (int32_t)rt->pts.count - 1, &seg_last)) {
                liste_update_relative_position(&user_history, &seg_last);
                rt->cur_time = user_history.pos_rel.time - rt->start_time;
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
    liste_init(&user_history);

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
        char path[64];
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

    for (uint16_t i = 0U; (i < segment_count) && (count < max_count); i++) {
        if ((segments[i].status == SEG_START) ||
            (segments[i].status == SEG_ON) ||
            (segments[i].status == SEG_FIN)) {
            segs[count] = segments[i];
            count++;
        }
    }

    return count;
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
    float nearest = -1.0f;

    /* This would need current position and segment start points */
    /* Placeholder implementation */

    return nearest;
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
