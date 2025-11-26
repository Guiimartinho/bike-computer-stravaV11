/**
 * @file suffer_score.c
 * @brief Suffer Score calculation implementation
 *
 * Based on original SufferScore.cpp implementation.
 */

#include <string.h>
#include <zephyr/logging/log.h>

#include "model/suffer_score.h"

LOG_MODULE_REGISTER(suffer_score, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** HR zone limits (from original) */
#define HRM_Z1_LIM      80U
#define HRM_Z2_LIM      120U
#define HRM_Z3_LIM      144U
#define HRM_Z4_LIM      165U
#define HRM_Z5_LIM      176U

/** Points per hour for each zone (from original) */
#define HRM_Z1_PTS_PER_HOUR     (16.0f / 3600.0f)
#define HRM_Z2_PTS_PER_HOUR     (33.0f / 3600.0f)
#define HRM_Z3_PTS_PER_HOUR     (72.0f / 3600.0f)
#define HRM_Z4_PTS_PER_HOUR     (85.0f / 3600.0f)
#define HRM_Z5_PTS_PER_HOUR     (95.0f / 3600.0f)

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Update the calculated score
 */
static void update_score(suffer_score_t *ss)
{
    ss->score = ss->hrm_bins[0] * HRM_Z1_PTS_PER_HOUR;
    ss->score += ss->hrm_bins[1] * HRM_Z2_PTS_PER_HOUR;
    ss->score += ss->hrm_bins[2] * HRM_Z3_PTS_PER_HOUR;
    ss->score += ss->hrm_bins[3] * HRM_Z4_PTS_PER_HOUR;
    ss->score += ss->hrm_bins[4] * HRM_Z5_PTS_PER_HOUR;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void suffer_score_init(suffer_score_t *ss)
{
    if (ss == NULL) {
        return;
    }

    (void)memset(ss, 0, sizeof(suffer_score_t));

    LOG_INF("Suffer score initialized");
}

void suffer_score_reset(suffer_score_t *ss)
{
    if (ss == NULL) {
        return;
    }

    (void)memset(ss, 0, sizeof(suffer_score_t));
}

void suffer_score_add_hrm(suffer_score_t *ss, uint8_t bpm, uint32_t timestamp_ms)
{
    if (ss == NULL) {
        return;
    }

    /* First call - initialize timestamp */
    if (ss->last_timestamp == 0U) {
        ss->last_timestamp = timestamp_ms;
        return;
    }

    /* Calculate elapsed time */
    float time_delta = (float)(timestamp_ms - ss->last_timestamp) / 1000.0f;

    if (time_delta <= 0.0f) {
        return;
    }

    /* Classify into zones (from original logic) */
    if ((bpm > HRM_Z1_LIM) && (bpm <= HRM_Z2_LIM)) {
        ss->hrm_bins[0] += time_delta;
    } else if ((bpm > HRM_Z2_LIM) && (bpm <= HRM_Z3_LIM)) {
        ss->hrm_bins[1] += time_delta;
    } else if ((bpm > HRM_Z3_LIM) && (bpm <= HRM_Z4_LIM)) {
        ss->hrm_bins[2] += time_delta;
    } else if ((bpm > HRM_Z4_LIM) && (bpm <= HRM_Z5_LIM)) {
        ss->hrm_bins[3] += time_delta;
    } else if (bpm > HRM_Z5_LIM) {
        ss->hrm_bins[4] += time_delta;
    } else {
        /* Below Z1 - not counted */
    }

    /* Update timestamp */
    ss->last_timestamp = timestamp_ms;

    /* Recalculate score */
    update_score(ss);
}

float suffer_score_get(const suffer_score_t *ss)
{
    if (ss == NULL) {
        return 0.0f;
    }

    return ss->score;
}

uint32_t suffer_score_get_zone_time(const suffer_score_t *ss, uint8_t zone)
{
    if ((ss == NULL) || (zone >= HRM_ZONES_NB)) {
        return 0U;
    }

    return (uint32_t)ss->hrm_bins[zone];
}

uint32_t suffer_score_get_total_time(const suffer_score_t *ss)
{
    if (ss == NULL) {
        return 0U;
    }

    float total = 0.0f;

    for (uint8_t i = 0U; i < HRM_ZONES_NB; i++) {
        total += ss->hrm_bins[i];
    }

    return (uint32_t)total;
}
