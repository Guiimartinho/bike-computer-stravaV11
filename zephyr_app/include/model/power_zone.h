/**
 * @file power_zone.h
 * @brief Power zone tracking based on FTP
 * @note Follows MISRA C:2012 guidelines
 *
 * Power zones (as percentage of FTP):
 * Z1: < 55%   - Active Recovery
 * Z2: 55-75%  - Endurance
 * Z3: 75-90%  - Tempo
 * Z4: 90-105% - Threshold
 * Z5: 105-120% - VO2max
 * Z6: 120-150% - Anaerobic
 * Z7: > 150%  - Neuromuscular
 */

#ifndef MODEL_POWER_ZONE_H_
#define MODEL_POWER_ZONE_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Number of power zones */
#define PW_ZONES_NB     7U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Power zone data structure
 */
typedef struct {
    float pw_bins[PW_ZONES_NB];     /**< Time in each zone (seconds) */
    uint32_t last_timestamp;         /**< Last update timestamp */
    uint8_t last_bin;                /**< Last zone index */
    uint16_t ftp;                    /**< Functional Threshold Power */
} power_zone_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize power zone tracker
 * @param pz Pointer to power zone structure
 * @param ftp Functional Threshold Power in watts
 */
void power_zone_init(power_zone_t *pz, uint16_t ftp);

/**
 * @brief Reset power zone data
 * @param pz Pointer to power zone structure
 */
void power_zone_reset(power_zone_t *pz);

/**
 * @brief Add power measurement
 * @param pz Pointer to power zone structure
 * @param power_watts Power measurement in watts
 * @param timestamp_ms Current timestamp in milliseconds
 */
void power_zone_add_data(power_zone_t *pz, uint16_t power_watts, uint32_t timestamp_ms);

/**
 * @brief Get time in specific zone
 * @param pz Pointer to power zone structure
 * @param zone Zone index (0-6)
 * @return Time in zone in seconds
 */
uint32_t power_zone_get_time(const power_zone_t *pz, uint8_t zone);

/**
 * @brief Get total tracked time
 * @param pz Pointer to power zone structure
 * @return Total time in seconds
 */
uint32_t power_zone_get_total_time(const power_zone_t *pz);

/**
 * @brief Get maximum time in any zone
 * @param pz Pointer to power zone structure
 * @return Maximum time in seconds
 */
uint32_t power_zone_get_max_time(const power_zone_t *pz);

/**
 * @brief Get current zone
 * @param pz Pointer to power zone structure
 * @return Current zone index
 */
uint8_t power_zone_get_current(const power_zone_t *pz);

/**
 * @brief Get number of zones
 * @return Number of power zones
 */
uint8_t power_zone_get_count(void);

/**
 * @brief Update FTP value
 * @param pz Pointer to power zone structure
 * @param ftp New FTP value
 */
void power_zone_set_ftp(power_zone_t *pz, uint16_t ftp);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_POWER_ZONE_H_ */
