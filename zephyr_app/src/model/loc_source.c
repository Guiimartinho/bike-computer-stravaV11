/**
 * @file loc_source.c
 * @brief Multiple location source management implementation
 *
 * Based on original Locator.cpp getUpdateSource() logic:
 *
 * Priority:
 * 1. SIM (Simulation) - Highest priority (Zwift mode)
 * 2. GPS (Internal)   - Primary source for outdoor
 * 3. LNS (BLE)        - Fallback when GPS has no fix
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "model/loc_source.h"

LOG_MODULE_REGISTER(loc_source, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Check if source data is recent enough
 */
static bool is_source_fresh(const loc_source_data_t *src,
                            uint32_t max_age_ms,
                            uint32_t current_time)
{
    if (!src->is_valid) {
        return false;
    }

    if (src->update_time == 0U) {
        return false;
    }

    uint32_t age = current_time - src->update_time;
    return (age <= max_age_ms);
}

/**
 * @brief Check if location data is valid
 */
static bool is_location_valid(const loc_data_t *loc)
{
    if (loc == NULL) {
        return false;
    }

    /* Check for null island */
    if ((loc->lat == 0.0f) && (loc->lon == 0.0f)) {
        return false;
    }

    /* Check reasonable ranges */
    if ((loc->lat < -90.0f) || (loc->lat > 90.0f)) {
        return false;
    }

    if ((loc->lon < -180.0f) || (loc->lon > 180.0f)) {
        return false;
    }

    return true;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void loc_source_init(loc_source_mgr_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    (void)memset(mgr, 0, sizeof(loc_source_mgr_t));
    mgr->active_source = LOC_SOURCE_NONE;
    mgr->is_initialized = true;

    LOG_INF("Location source manager initialized");
}

void loc_source_reset(loc_source_mgr_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    (void)memset(&mgr->gps, 0, sizeof(loc_source_data_t));
    (void)memset(&mgr->lns, 0, sizeof(loc_source_data_t));
    (void)memset(&mgr->sim, 0, sizeof(loc_source_data_t));
    mgr->active_source = LOC_SOURCE_NONE;

    LOG_INF("Location sources reset");
}

void loc_source_update_gps(loc_source_mgr_t *mgr,
                           const loc_data_t *loc,
                           const date_data_t *date,
                           uint32_t timestamp_ms)
{
    if ((mgr == NULL) || (loc == NULL)) {
        return;
    }

    if (!is_location_valid(loc)) {
        return;
    }

    mgr->gps.loc = *loc;
    mgr->gps.update_time = timestamp_ms;
    mgr->gps.is_valid = true;

    if (date != NULL) {
        mgr->gps.date = *date;
    }

    LOG_DBG("GPS updated: %.6f, %.6f", (double)loc->lat, (double)loc->lon);
}

void loc_source_update_lns(loc_source_mgr_t *mgr,
                           const loc_data_t *loc,
                           uint32_t timestamp_ms)
{
    if ((mgr == NULL) || (loc == NULL)) {
        return;
    }

    if (!is_location_valid(loc)) {
        return;
    }

    mgr->lns.loc = *loc;
    mgr->lns.update_time = timestamp_ms;
    mgr->lns.is_valid = true;

    LOG_DBG("LNS updated: %.6f, %.6f", (double)loc->lat, (double)loc->lon);
}

void loc_source_update_sim(loc_source_mgr_t *mgr,
                           const loc_data_t *loc,
                           uint32_t timestamp_ms)
{
    if ((mgr == NULL) || (loc == NULL)) {
        return;
    }

    if (!is_location_valid(loc)) {
        return;
    }

    mgr->sim.loc = *loc;
    mgr->sim.update_time = timestamp_ms;
    mgr->sim.is_valid = true;

    LOG_DBG("SIM updated: %.6f, %.6f", (double)loc->lat, (double)loc->lon);
}

bool loc_source_get_best(loc_source_mgr_t *mgr,
                         loc_data_t *loc,
                         loc_source_t *source,
                         uint32_t timestamp_ms)
{
    if ((mgr == NULL) || (loc == NULL)) {
        return false;
    }

    loc_source_t selected = LOC_SOURCE_NONE;

    /* Priority 1: Simulation (Zwift mode)
     * If SIM source is fresh, use it exclusively
     */
    if (is_source_fresh(&mgr->sim, LOC_SOURCE_SIM_MAX_AGE_MS, timestamp_ms)) {
        *loc = mgr->sim.loc;
        selected = LOC_SOURCE_SIM;
        LOG_DBG("Using SIM source");
    }
    /* Priority 2: GPS (Internal)
     * Primary source for outdoor cycling
     */
    else if (is_source_fresh(&mgr->gps, LOC_SOURCE_GPS_MAX_AGE_MS, timestamp_ms)) {
        *loc = mgr->gps.loc;
        selected = LOC_SOURCE_GPS;
        LOG_DBG("Using GPS source");
    }
    /* Priority 3: LNS (BLE)
     * Fallback when GPS has no fix - uses phone's GPS via BLE
     * Only used if GPS is not available
     */
    else if (is_source_fresh(&mgr->lns, LOC_SOURCE_LNS_MAX_AGE_MS, timestamp_ms)) {
        /* Check if GPS is trying to acquire (not completely unavailable) */
        bool gps_acquiring = (mgr->gps.update_time > 0U) &&
                            ((timestamp_ms - mgr->gps.update_time) < 30000U);

        if (!gps_acquiring) {
            *loc = mgr->lns.loc;
            selected = LOC_SOURCE_BLE_LNS;
            LOG_DBG("Using LNS source (GPS unavailable)");
        }
    }

    mgr->active_source = selected;
    mgr->last_check_time = timestamp_ms;

    if (source != NULL) {
        *source = selected;
    }

    return (selected != LOC_SOURCE_NONE);
}

loc_source_t loc_source_get_active(const loc_source_mgr_t *mgr)
{
    if (mgr == NULL) {
        return LOC_SOURCE_NONE;
    }

    return mgr->active_source;
}

bool loc_source_is_available(const loc_source_mgr_t *mgr,
                             loc_source_t source,
                             uint32_t timestamp_ms)
{
    if (mgr == NULL) {
        return false;
    }

    switch (source) {
    case LOC_SOURCE_GPS:
        return is_source_fresh(&mgr->gps, LOC_SOURCE_GPS_MAX_AGE_MS, timestamp_ms);

    case LOC_SOURCE_BLE_LNS:
        return is_source_fresh(&mgr->lns, LOC_SOURCE_LNS_MAX_AGE_MS, timestamp_ms);

    case LOC_SOURCE_SIM:
        return is_source_fresh(&mgr->sim, LOC_SOURCE_SIM_MAX_AGE_MS, timestamp_ms);

    default:
        return false;
    }
}

uint32_t loc_source_get_age(const loc_source_mgr_t *mgr,
                            loc_source_t source,
                            uint32_t timestamp_ms)
{
    if (mgr == NULL) {
        return UINT32_MAX;
    }

    const loc_source_data_t *src = NULL;

    switch (source) {
    case LOC_SOURCE_GPS:
        src = &mgr->gps;
        break;

    case LOC_SOURCE_BLE_LNS:
        src = &mgr->lns;
        break;

    case LOC_SOURCE_SIM:
        src = &mgr->sim;
        break;

    default:
        return UINT32_MAX;
    }

    if (!src->is_valid || (src->update_time == 0U)) {
        return UINT32_MAX;
    }

    return timestamp_ms - src->update_time;
}

bool loc_source_gps_has_fix(const loc_source_mgr_t *mgr)
{
    if (mgr == NULL) {
        return false;
    }

    return mgr->gps.is_valid;
}

bool loc_source_is_sim_mode(const loc_source_mgr_t *mgr,
                            uint32_t timestamp_ms)
{
    if (mgr == NULL) {
        return false;
    }

    return is_source_fresh(&mgr->sim, LOC_SOURCE_SIM_MAX_AGE_MS, timestamp_ms);
}
