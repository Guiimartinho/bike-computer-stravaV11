/**
 * @file suffer_score.h
 * @brief Suffer Score calculation based on heart rate zones
 * @note Follows MISRA C:2012 guidelines
 *
 * Suffer Score is calculated by accumulating time in each HR zone
 * and applying zone-specific multipliers.
 *
 * HR Zones (from original):
 * Z1: 80-120 bpm  - 16 pts/hour
 * Z2: 120-144 bpm - 33 pts/hour
 * Z3: 144-165 bpm - 72 pts/hour
 * Z4: 165-176 bpm - 85 pts/hour
 * Z5: >176 bpm    - 95 pts/hour
 */

#ifndef MODEL_SUFFER_SCORE_H_
#define MODEL_SUFFER_SCORE_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Number of HR zones */
#define HRM_ZONES_NB    5U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Suffer score data structure
 */
typedef struct {
    float hrm_bins[HRM_ZONES_NB];   /**< Time in each zone (seconds) */
    float score;                     /**< Calculated suffer score */
    uint32_t last_timestamp;         /**< Last update timestamp */
} suffer_score_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize suffer score tracker
 * @param ss Pointer to suffer score structure
 */
void suffer_score_init(suffer_score_t *ss);

/**
 * @brief Reset suffer score data
 * @param ss Pointer to suffer score structure
 */
void suffer_score_reset(suffer_score_t *ss);

/**
 * @brief Add heart rate measurement
 * @param ss Pointer to suffer score structure
 * @param bpm Heart rate in beats per minute
 * @param timestamp_ms Current timestamp in milliseconds
 */
void suffer_score_add_hrm(suffer_score_t *ss, uint8_t bpm, uint32_t timestamp_ms);

/**
 * @brief Get current suffer score
 * @param ss Pointer to suffer score structure
 * @return Suffer score value
 */
float suffer_score_get(const suffer_score_t *ss);

/**
 * @brief Get time in specific HR zone
 * @param ss Pointer to suffer score structure
 * @param zone Zone index (0-4)
 * @return Time in zone in seconds
 */
uint32_t suffer_score_get_zone_time(const suffer_score_t *ss, uint8_t zone);

/**
 * @brief Get total tracked time
 * @param ss Pointer to suffer score structure
 * @return Total time in seconds
 */
uint32_t suffer_score_get_total_time(const suffer_score_t *ss);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_SUFFER_SCORE_H_ */
