/**
 * @file ble_komoot_client.h
 * @brief BLE Komoot Navigation Service Client
 *
 * GATT client for Komoot BLE Connect navigation service.
 * Receives turn-by-turn navigation instructions from Komoot app.
 *
 * Follows MISRA C:2012 guidelines.
 */

#ifndef RF_BLE_KOMOOT_CLIENT_H
#define RF_BLE_KOMOOT_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** Komoot Service UUID (vendor specific) */
#define BLE_UUID_KOMOOT_SERVICE         0x6001U

/** Komoot Navigation Characteristic UUID */
#define BLE_UUID_KOMOOT_NAV_CHAR        0x6002U

/** Komoot Position Notification UUID */
#define BLE_UUID_KOMOOT_POS_CHAR        0x6003U

/* ==========================================================================
 * Navigation Direction Icons
 * ========================================================================== */

/** Navigation direction enumeration (Komoot protocol) */
typedef enum {
    KOMOOT_DIR_NONE             = 0,    /**< No direction */
    KOMOOT_DIR_STRAIGHT         = 1,    /**< Continue straight */
    KOMOOT_DIR_START            = 2,    /**< Route start */
    KOMOOT_DIR_FINISH           = 3,    /**< Route finish */
    KOMOOT_DIR_SLIGHT_LEFT      = 4,    /**< Slight left */
    KOMOOT_DIR_LEFT             = 5,    /**< Turn left */
    KOMOOT_DIR_SHARP_LEFT       = 6,    /**< Sharp left */
    KOMOOT_DIR_SLIGHT_RIGHT     = 7,    /**< Slight right */
    KOMOOT_DIR_RIGHT            = 8,    /**< Turn right */
    KOMOOT_DIR_SHARP_RIGHT      = 9,    /**< Sharp right */
    KOMOOT_DIR_FORK_LEFT        = 10,   /**< Fork left */
    KOMOOT_DIR_FORK_RIGHT       = 11,   /**< Fork right */
    KOMOOT_DIR_U_TURN           = 12,   /**< U-turn */
    KOMOOT_DIR_U_TURN_LEFT      = 13,   /**< U-turn left */
    KOMOOT_DIR_U_TURN_RIGHT     = 14,   /**< U-turn right */
    KOMOOT_DIR_ROUNDABOUT_EXIT1 = 15,   /**< Roundabout exit 1 */
    KOMOOT_DIR_ROUNDABOUT_EXIT2 = 16,   /**< Roundabout exit 2 */
    KOMOOT_DIR_ROUNDABOUT_EXIT3 = 17,   /**< Roundabout exit 3 */
    KOMOOT_DIR_ROUNDABOUT_CCW1  = 18,   /**< Roundabout CCW exit 1 */
    KOMOOT_DIR_ROUNDABOUT_CCW2  = 19,   /**< Roundabout CCW exit 2 */
    KOMOOT_DIR_ROUNDABOUT_CCW3  = 20,   /**< Roundabout CCW exit 3 */
    KOMOOT_DIR_ROUNDABOUT_FALL  = 21,   /**< Roundabout fallback */
    KOMOOT_DIR_OUT_OF_ROUTE     = 22,   /**< Out of route */
    KOMOOT_DIR_FERRY            = 23,   /**< Take ferry */
    KOMOOT_DIR_MAX              = 24    /**< Direction count */
} komoot_direction_t;

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Komoot navigation data structure */
typedef struct {
    bool is_updated;            /**< True if data has been updated */
    bool is_connected;          /**< True if connected to Komoot service */
    komoot_direction_t direction;  /**< Current navigation direction */
    uint32_t distance;          /**< Distance to next turn in meters */
    char street_name[32];       /**< Street name (if available) */
    uint32_t timestamp;         /**< Last update timestamp */
} komoot_nav_t;

/** Komoot client state enumeration */
typedef enum {
    KOMOOT_STATE_IDLE = 0,
    KOMOOT_STATE_DISCOVERING,
    KOMOOT_STATE_CONNECTED,
    KOMOOT_STATE_SUBSCRIBED
} komoot_client_state_t;

/** Komoot event callback type */
typedef void (*komoot_nav_callback_t)(const komoot_nav_t *nav);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize Komoot client
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_komoot_client_init(void);

/**
 * @brief Register navigation callback
 * @param callback Function to call on navigation update
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_komoot_client_register_callback(komoot_nav_callback_t callback);

/**
 * @brief Get current navigation data
 * @param nav Pointer to store navigation data
 * @return APP_OK on success, APP_ERR_NOT_FOUND if no data
 */
app_err_t ble_komoot_client_get_nav(komoot_nav_t *nav);

/**
 * @brief Check if connected to Komoot service
 * @return true if connected and subscribed
 */
bool ble_komoot_client_is_connected(void);

/**
 * @brief Clear update flag after reading
 */
void ble_komoot_client_clear_update(void);

/**
 * @brief Get navigation icon for direction
 * @param direction Navigation direction
 * @return Pointer to icon bitmap (32x32 1-bit), or NULL if invalid
 */
const uint8_t *ble_komoot_get_icon(komoot_direction_t direction);

/**
 * @brief Get direction name string
 * @param direction Navigation direction
 * @return Direction name string
 */
const char *ble_komoot_get_direction_name(komoot_direction_t direction);

/**
 * @brief Handle BLE connection for Komoot discovery
 * @param conn Connection handle
 */
void ble_komoot_client_on_connect(struct bt_conn *conn);

/**
 * @brief Handle BLE disconnection
 * @param conn Connection handle
 */
void ble_komoot_client_on_disconnect(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_KOMOOT_CLIENT_H */
