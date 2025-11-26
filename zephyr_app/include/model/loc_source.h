/**
 * @file loc_source.h
 * @brief Multiple location source management
 *
 * Handles prioritization and fusion of multiple position sources:
 * - GPS (internal GPS module)
 * - LNS (BLE Location and Navigation Service)
 * - SIM (Simulation via BLE NUS - Zwift mode)
 *
 * Based on original Locator.cpp implementation.
 */

#ifndef MODEL_LOC_SOURCE_H
#define MODEL_LOC_SOURCE_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum age for GPS source to be considered valid (ms) */
#define LOC_SOURCE_GPS_MAX_AGE_MS       1500U

/** Maximum age for LNS source to be considered valid (ms) */
#define LOC_SOURCE_LNS_MAX_AGE_MS       2000U

/** Maximum age for SIM source to be considered valid (ms) */
#define LOC_SOURCE_SIM_MAX_AGE_MS       2000U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Location source data with timestamp
 */
typedef struct {
    loc_data_t loc;             /**< Location data */
    date_data_t date;           /**< Date/time data */
    uint32_t update_time;       /**< Last update timestamp (ms) */
    bool is_valid;              /**< Data validity flag */
} loc_source_data_t;

/**
 * @brief Location source manager state
 */
typedef struct {
    loc_source_data_t gps;      /**< GPS source */
    loc_source_data_t lns;      /**< BLE LNS source */
    loc_source_data_t sim;      /**< Simulation source */
    loc_source_t active_source; /**< Currently active source */
    uint32_t last_check_time;   /**< Last priority check time */
    bool is_initialized;        /**< Initialization flag */
} loc_source_mgr_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize location source manager
 * @param mgr Pointer to manager state
 */
void loc_source_init(loc_source_mgr_t *mgr);

/**
 * @brief Reset all location sources
 * @param mgr Pointer to manager state
 */
void loc_source_reset(loc_source_mgr_t *mgr);

/**
 * @brief Update GPS location source
 * @param mgr Pointer to manager state
 * @param loc Location data
 * @param date Date/time data (can be NULL)
 * @param timestamp_ms Update timestamp
 */
void loc_source_update_gps(loc_source_mgr_t *mgr,
                           const loc_data_t *loc,
                           const date_data_t *date,
                           uint32_t timestamp_ms);

/**
 * @brief Update BLE LNS location source
 * @param mgr Pointer to manager state
 * @param loc Location data
 * @param timestamp_ms Update timestamp
 */
void loc_source_update_lns(loc_source_mgr_t *mgr,
                           const loc_data_t *loc,
                           uint32_t timestamp_ms);

/**
 * @brief Update simulation location source
 * @param mgr Pointer to manager state
 * @param loc Location data
 * @param timestamp_ms Update timestamp
 */
void loc_source_update_sim(loc_source_mgr_t *mgr,
                           const loc_data_t *loc,
                           uint32_t timestamp_ms);

/**
 * @brief Get best available location
 *
 * Priority: SIM > GPS > LNS (when GPS has no fix)
 *
 * @param mgr Pointer to manager state
 * @param loc Output location data
 * @param source Output source type (can be NULL)
 * @param timestamp_ms Current timestamp
 * @return true if valid location available
 */
bool loc_source_get_best(loc_source_mgr_t *mgr,
                         loc_data_t *loc,
                         loc_source_t *source,
                         uint32_t timestamp_ms);

/**
 * @brief Get current active source
 * @param mgr Pointer to manager state
 * @return Active source type
 */
loc_source_t loc_source_get_active(const loc_source_mgr_t *mgr);

/**
 * @brief Check if specific source is available
 * @param mgr Pointer to manager state
 * @param source Source to check
 * @param timestamp_ms Current timestamp
 * @return true if source has recent valid data
 */
bool loc_source_is_available(const loc_source_mgr_t *mgr,
                             loc_source_t source,
                             uint32_t timestamp_ms);

/**
 * @brief Get age of specific source data
 * @param mgr Pointer to manager state
 * @param source Source to check
 * @param timestamp_ms Current timestamp
 * @return Age in milliseconds, or UINT32_MAX if invalid
 */
uint32_t loc_source_get_age(const loc_source_mgr_t *mgr,
                            loc_source_t source,
                            uint32_t timestamp_ms);

/**
 * @brief Check if GPS has fix
 * @param mgr Pointer to manager state
 * @return true if GPS source is valid
 */
bool loc_source_gps_has_fix(const loc_source_mgr_t *mgr);

/**
 * @brief Check if in simulation mode
 * @param mgr Pointer to manager state
 * @param timestamp_ms Current timestamp
 * @return true if simulation source is active
 */
bool loc_source_is_sim_mode(const loc_source_mgr_t *mgr,
                            uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_LOC_SOURCE_H */
