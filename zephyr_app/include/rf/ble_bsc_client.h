/**
 * @file ble_bsc_client.h
 * @brief BLE Cycling Speed and Cadence Service Client
 *
 * Implements GATT client for CSC Service (0x1816)
 */

#ifndef BLE_BSC_CLIENT_H
#define BLE_BSC_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

/* Forward declaration */
struct bt_conn;

#ifdef __cplusplus
extern "C" {
#endif

/** BSC Client state */
typedef enum {
    BSC_CLIENT_STATE_IDLE = 0,
    BSC_CLIENT_STATE_DISCOVERING,
    BSC_CLIENT_STATE_CONNECTED,
    BSC_CLIENT_STATE_SUBSCRIBED
} bsc_client_state_t;

/** BSC data callback - speed in 0.01 km/h, cadence in RPM */
typedef void (*bsc_data_callback_t)(uint16_t speed, uint8_t cadence);

/** Connection state callback */
typedef void (*bsc_conn_callback_t)(bool connected);

/**
 * @brief Initialize BSC client
 * @return APP_OK on success
 */
app_err_t ble_bsc_client_init(void);

/**
 * @brief Register data callback
 * @param callback Function to call when BSC data received
 */
void ble_bsc_client_register_callback(bsc_data_callback_t callback);

/**
 * @brief Register connection callback
 * @param callback Function to call on connect/disconnect
 */
void ble_bsc_client_register_conn_callback(bsc_conn_callback_t callback);

/**
 * @brief Check if BSC device is connected
 * @return true if connected and subscribed
 */
bool ble_bsc_client_is_connected(void);

/**
 * @brief Get current speed
 * @return Speed in 0.01 km/h or 0 if not connected
 */
uint16_t ble_bsc_client_get_speed(void);

/**
 * @brief Get current cadence
 * @return Cadence in RPM or 0 if not connected
 */
uint8_t ble_bsc_client_get_cadence(void);

/**
 * @brief Get BSC info structure
 * @param info Pointer to bsc_info_t to fill
 * @return APP_OK on success
 */
app_err_t ble_bsc_client_get_info(bsc_info_t *info);

/**
 * @brief Set wheel circumference for speed calculation
 * @param circumference_mm Wheel circumference in millimeters
 */
void ble_bsc_client_set_wheel_circumference(uint16_t circumference_mm);

/**
 * @brief Handle connection event (called by ble_manager)
 * @param conn Connection handle
 */
void ble_bsc_client_on_connect(struct bt_conn *conn);

/**
 * @brief Handle disconnection event (called by ble_manager)
 * @param conn Connection handle
 */
void ble_bsc_client_on_disconnect(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif /* BLE_BSC_CLIENT_H */
