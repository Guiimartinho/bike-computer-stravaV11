/**
 * @file ble_nus.h
 * @brief Nordic UART Service wrapper for stravaV10
 *
 * Provides serial communication over BLE.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef RF_BLE_NUS_H
#define RF_BLE_NUS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** NUS receive callback */
typedef void (*ble_nus_rx_callback_t)(const uint8_t *data, uint16_t len);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize NUS service
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_nus_init(void);

/**
 * @brief Send data via NUS
 * @param data Data buffer
 * @param len Data length
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_nus_send(const uint8_t *data, uint16_t len);

/**
 * @brief Send string via NUS
 * @param str Null-terminated string
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_nus_send_str(const char *str);

/**
 * @brief Register receive callback
 * @param callback Function to call when data is received
 * @return APP_OK on success, error code otherwise
 */
app_err_t ble_nus_register_callback(ble_nus_rx_callback_t callback);

/**
 * @brief Check if NUS is connected and ready
 * @return true if ready to send
 */
bool ble_nus_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_NUS_H */
