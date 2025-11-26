/**
 * @file rr_zone.h
 * @brief RR interval (HRV) zone tracking
 * @note Follows MISRA C:2012 guidelines
 *
 * Tracks RMSSD (Root Mean Square of Successive Differences)
 * for Heart Rate Variability analysis by HR zone.
 */

#ifndef MODEL_RR_ZONE_H_
#define MODEL_RR_ZONE_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Number of RR zones (by HR) */
#define RR_ZONES_NB         6U

/** Number of samples for RMSSD calculation */
#define RR_VAR_NB_ELEM      20U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief RR zone data structure
 */
typedef struct {
    float rr_bins[RR_ZONES_NB];     /**< Accumulated RMSSD by zone */
    float tm_bins[RR_ZONES_NB];     /**< Time count by zone */
    float rr_buffer[RR_VAR_NB_ELEM]; /**< RR interval buffer for RMSSD */
    uint16_t buffer_count;           /**< Current buffer count */
    uint32_t last_timestamp;         /**< Last update timestamp */
    uint8_t last_bin;                /**< Last zone index */
} rr_zone_t;

/* Note: hrm_info_t is defined in app_types.h */

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize RR zone tracker
 * @param rz Pointer to RR zone structure
 */
void rr_zone_init(rr_zone_t *rz);

/**
 * @brief Reset RR zone data
 * @param rz Pointer to RR zone structure
 */
void rr_zone_reset(rr_zone_t *rz);

/**
 * @brief Add RR interval data
 * @param rz Pointer to RR zone structure
 * @param hrm_info HRM information with RR interval
 */
void rr_zone_add_data(rr_zone_t *rz, const hrm_info_t *hrm_info);

/**
 * @brief Get average RMSSD value for zone
 * @param rz Pointer to RR zone structure
 * @param zone Zone index (0-5)
 * @return Average RMSSD value
 */
float rr_zone_get_value(const rr_zone_t *rz, uint8_t zone);

/**
 * @brief Get time count in zone
 * @param rz Pointer to RR zone structure
 * @param zone Zone index (0-5)
 * @return Time count
 */
uint32_t rr_zone_get_time(const rr_zone_t *rz, uint8_t zone);

/**
 * @brief Get total time tracked
 * @param rz Pointer to RR zone structure
 * @return Total time
 */
uint32_t rr_zone_get_total_time(const rr_zone_t *rz);

/**
 * @brief Get maximum RMSSD value across zones
 * @param rz Pointer to RR zone structure
 * @return Maximum RMSSD value
 */
float rr_zone_get_max_value(const rr_zone_t *rz);

/**
 * @brief Get maximum time in any zone
 * @param rz Pointer to RR zone structure
 * @return Maximum time
 */
uint32_t rr_zone_get_max_time(const rr_zone_t *rz);

/**
 * @brief Get current zone
 * @param rz Pointer to RR zone structure
 * @return Current zone index
 */
uint8_t rr_zone_get_current(const rr_zone_t *rz);

/**
 * @brief Get number of zones
 * @return Number of RR zones
 */
uint8_t rr_zone_get_count(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_RR_ZONE_H_ */
