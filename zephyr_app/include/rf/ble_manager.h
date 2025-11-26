/**
 * @file ble_manager.h
 * @brief BLE Manager for stravaV10
 *
 * Manages BLE peripheral and central connections.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef RF_BLE_MANAGER_H
#define RF_BLE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** BLE connection state */
typedef enum {
    BLE_STATE_IDLE = 0,
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTED,
    BLE_STATE_SCANNING,
    BLE_STATE_CONNECTING
} ble_state_t;

/** BLE connection info */
typedef struct {
    uint8_t addr[6];        /**< Peer address */
    uint16_t conn_handle;   /**< Connection handle */
    int8_t rssi;            /**< Signal strength */
    bool connected;         /**< Connection state */
} ble_conn_info_t;

/** BLE event callback type */
typedef void (*ble_event_callback_t)(uint8_t event, void *data);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize BLE manager
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_init(void);

/**
 * @brief Start BLE advertising
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_start_advertising(void);

/**
 * @brief Stop BLE advertising
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_stop_advertising(void);

/**
 * @brief Start BLE scanning for sensors
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_start_scan(void);

/**
 * @brief Stop BLE scanning
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_stop_scan(void);

/**
 * @brief Get current BLE state
 * @return Current state
 */
ble_state_t ble_manager_get_state(void);

/**
 * @brief Check if any device is connected
 * @return true if connected
 */
bool ble_manager_is_connected(void);

/**
 * @brief Get connection info
 * @param info Pointer to store connection info
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_get_conn_info(ble_conn_info_t *info);

/**
 * @brief Disconnect current connection
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_disconnect(void);

/**
 * @brief Register event callback
 * @param callback Function to call on BLE events
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_register_callback(ble_event_callback_t callback);

/**
 * @brief Send data via Nordic UART Service
 * @param data Data buffer
 * @param len Data length
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_manager_nus_send(const uint8_t *data, uint16_t len);

/**
 * @brief Update battery level characteristic
 * @param level Battery level (0-100)
 */
void ble_manager_update_battery(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_MANAGER_H */
