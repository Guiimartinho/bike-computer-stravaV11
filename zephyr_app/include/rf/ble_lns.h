/**
 * @file ble_lns.h
 * @brief BLE Location and Navigation Service for stravaV10
 *
 * Implements the LNS for transmitting GPS data to connected devices.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef RF_BLE_LNS_H
#define RF_BLE_LNS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** LNS Location and Speed flags */
typedef enum {
    LNS_FLAG_SPEED_PRESENT      = (1U << 0),
    LNS_FLAG_DISTANCE_PRESENT   = (1U << 1),
    LNS_FLAG_LOCATION_PRESENT   = (1U << 2),
    LNS_FLAG_ELEVATION_PRESENT  = (1U << 3),
    LNS_FLAG_HEADING_PRESENT    = (1U << 4),
    LNS_FLAG_ROLLING_TIME       = (1U << 5),
    LNS_FLAG_UTC_TIME           = (1U << 6),
    LNS_FLAG_POSITION_STATUS    = (1U << 7),
} lns_flags_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize LNS service
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_lns_init(void);

/**
 * @brief Update location and speed
 * @param loc Location data
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_lns_update_location(const loc_data_t *loc);

/**
 * @brief Update position quality
 * @param hdop Horizontal DOP
 * @param vdop Vertical DOP
 * @param satellites Number of satellites
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_lns_update_quality(float hdop, float vdop, uint8_t satellites);

/**
 * @brief Check if notifications are enabled
 * @return true if notifications enabled
 */
bool ble_lns_notifications_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_LNS_H */
