/**
 * @file ant.h
 * @brief ANT+ Protocol Stub Interface
 *
 * IMPORTANT: ANT+ protocol is NOT available in Zephyr/NCS.
 *
 * ANT+ is a proprietary wireless protocol owned by Garmin/Dynastream.
 * It requires special licensing and the Nordic SoftDevice S340 which
 * combines both BLE and ANT stacks. This is not supported in Zephyr.
 *
 * ALTERNATIVES:
 * - Use BLE equivalents: HRS, CSC (Speed/Cadence), FTMS (Trainers)
 * - The stravaV10 Zephyr implementation uses BLE clients:
 *   - ble_hrs_client.h - Heart Rate Monitor (replaces ANT+ HRM)
 *   - ble_bsc_client.h - Bike Speed/Cadence (replaces ANT+ BSC)
 *   - ble_fec_client.h - Fitness Equipment (replaces ANT+ FE-C)
 *
 * This header provides stub functions for code compatibility only.
 * The actual functionality is handled by BLE clients.
 */

#ifndef RF_ANT_H
#define RF_ANT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * ANT+ Not Available Warning
 * ========================================================================== */

#ifndef ANT_STUB_WARNING_SHOWN
#define ANT_STUB_WARNING_SHOWN
#pragma message("ANT+ protocol is NOT available in Zephyr. Using BLE alternatives.")
#endif

/* ==========================================================================
 * Type Definitions (for compatibility)
 * ========================================================================== */

/** ANT channel numbers (stub - not actually used) */
typedef enum {
    ANT_CHANNEL_HRM = 0,
    ANT_CHANNEL_BSC = 1,
    ANT_CHANNEL_FEC = 2,
    ANT_CHANNEL_GLASSES = 3
} ant_channel_t;

/** ANT pairing sensor type (stub) */
typedef enum {
    ANT_PAIRING_NONE = 0,
    ANT_PAIRING_HRM,
    ANT_PAIRING_BSC,
    ANT_PAIRING_FEC
} ant_pairing_type_t;

/* ==========================================================================
 * Public Functions (Stubs)
 * ========================================================================== */

/**
 * @brief Initialize ANT stack (STUB - does nothing)
 * @return APP_OK always (ANT not available)
 * @note Use ble_manager_init() instead for BLE sensors
 */
static inline app_err_t ant_stack_init(void)
{
    /* ANT+ not available in Zephyr - use BLE alternatives */
    return APP_OK;
}

/**
 * @brief Setup ANT profiles (STUB - does nothing)
 * @return APP_OK always
 * @note Use ble_hrs_client_init(), ble_bsc_client_init() instead
 */
static inline app_err_t ant_setup_init(void)
{
    return APP_OK;
}

/**
 * @brief Start ANT with device IDs (STUB - does nothing)
 * @param hrm_id HRM device ID (ignored)
 * @param bsc_id BSC device ID (ignored)
 * @param fec_id FEC device ID (ignored)
 * @note Use ble_manager_start_scan() to connect to BLE sensors
 */
static inline void ant_setup_start(uint16_t hrm_id, uint16_t bsc_id, uint16_t fec_id)
{
    (void)hrm_id;
    (void)bsc_id;
    (void)fec_id;
}

/**
 * @brief Start ANT search (STUB - does nothing)
 * @param search_type Type of sensor to search for
 * @note Use ble_manager_start_scan() instead
 */
static inline void ant_search_start(ant_pairing_type_t search_type)
{
    (void)search_type;
}

/**
 * @brief End ANT search (STUB - does nothing)
 * @param search_type Type of sensor
 * @param dev_id Device ID found
 */
static inline void ant_search_end(ant_pairing_type_t search_type, uint16_t dev_id)
{
    (void)search_type;
    (void)dev_id;
}

/**
 * @brief ANT tasks processing (STUB - does nothing)
 * @note BLE processing is handled automatically by Zephyr
 */
static inline void ant_tasks(void)
{
    /* Nothing to do - BLE processing is automatic */
}

/**
 * @brief Initialize ANT timers (STUB - does nothing)
 */
static inline void ant_timers_init(void)
{
    /* Nothing to do */
}

#ifdef __cplusplus
}
#endif

#endif /* RF_ANT_H */
