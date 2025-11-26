/**
 * @file ble_hrs_client.h
 * @brief BLE Heart Rate Service Client
 *
 * Implements GATT client for Heart Rate Service (0x180D)
 */

#ifndef BLE_HRS_CLIENT_H
#define BLE_HRS_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

/* Forward declaration */
struct bt_conn;

#ifdef __cplusplus
extern "C" {
#endif

/** HRS Client state */
typedef enum {
    HRS_CLIENT_STATE_IDLE = 0,
    HRS_CLIENT_STATE_DISCOVERING,
    HRS_CLIENT_STATE_CONNECTED,
    HRS_CLIENT_STATE_SUBSCRIBED
} hrs_client_state_t;

/** Heart rate data callback */
typedef void (*hrs_data_callback_t)(uint8_t bpm, uint16_t rr_interval);

/** Connection state callback */
typedef void (*hrs_conn_callback_t)(bool connected);

/**
 * @brief Initialize HRS client
 * @return APP_OK on success
 */
app_err_t ble_hrs_client_init(void);

/**
 * @brief Register data callback
 * @param callback Function to call when HR data received
 */
void ble_hrs_client_register_callback(hrs_data_callback_t callback);

/**
 * @brief Register connection callback
 * @param callback Function to call on connect/disconnect
 */
void ble_hrs_client_register_conn_callback(hrs_conn_callback_t callback);

/**
 * @brief Check if HRS device is connected
 * @return true if connected and subscribed
 */
bool ble_hrs_client_is_connected(void);

/**
 * @brief Get current heart rate
 * @return Current BPM or 0 if not connected
 */
uint8_t ble_hrs_client_get_bpm(void);

/**
 * @brief Get last RR interval
 * @return RR interval in ms or 0 if not available
 */
uint16_t ble_hrs_client_get_rr(void);

/**
 * @brief Get HRS info structure
 * @param info Pointer to hrm_info_t to fill
 * @return APP_OK on success
 */
app_err_t ble_hrs_client_get_info(hrm_info_t *info);

/**
 * @brief Handle connection event (called by ble_manager)
 * @param conn Connection handle
 */
void ble_hrs_client_on_connect(struct bt_conn *conn);

/**
 * @brief Handle disconnection event (called by ble_manager)
 * @param conn Connection handle
 */
void ble_hrs_client_on_disconnect(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif /* BLE_HRS_CLIENT_H */
