/**
 * @file ble_fec_client.h
 * @brief BLE Fitness Machine Service Client (FE-C over BLE)
 *
 * Implements GATT client for FTMS (0x1826) for smart trainer control.
 */

#ifndef BLE_FEC_CLIENT_H
#define BLE_FEC_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

/* Forward declaration */
struct bt_conn;

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum grade in percent */
#define FEC_MAX_GRADE       25

/** Minimum grade in percent */
#define FEC_MIN_GRADE       (-25)

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** FEC Client state */
typedef enum {
    FEC_CLIENT_STATE_IDLE = 0,
    FEC_CLIENT_STATE_DISCOVERING,
    FEC_CLIENT_STATE_CONNECTED,
    FEC_CLIENT_STATE_SUBSCRIBED,
    FEC_CLIENT_STATE_CONTROLLING
} fec_client_state_t;

/** Trainer status */
typedef enum {
    TRAINER_STATUS_UNKNOWN = 0,
    TRAINER_STATUS_READY,
    TRAINER_STATUS_IN_USE,
    TRAINER_STATUS_PAUSED
} trainer_status_t;

/** FEC control mode */
typedef enum {
    FEC_MODE_SIMULATION = 0,    /**< Simulate grade/wind */
    FEC_MODE_ERG,               /**< Target power mode */
    FEC_MODE_RESISTANCE         /**< Manual resistance */
} fec_mode_t;

/** FEC data from trainer */
typedef struct {
    uint16_t power;             /**< Instantaneous power (W) */
    uint16_t speed;             /**< Speed in 0.01 km/h */
    uint8_t cadence;            /**< Cadence in RPM */
    int8_t grade;               /**< Current simulated grade */
    uint16_t target_power;      /**< Target power in ERG mode */
    uint8_t resistance;         /**< Resistance level (0-100) */
    trainer_status_t status;    /**< Trainer status */
    fec_mode_t mode;            /**< Current control mode */
    uint32_t elapsed_time;      /**< Elapsed time in seconds */
    uint32_t timestamp;         /**< Last update timestamp */
    bool connected;             /**< Connection status */
} fec_data_t;

/** FEC data callback */
typedef void (*fec_data_callback_t)(const fec_data_t *data);

/** Connection state callback */
typedef void (*fec_conn_callback_t)(bool connected);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize FEC client
 * @return APP_OK on success
 */
app_err_t ble_fec_client_init(void);

/**
 * @brief Register data callback
 * @param callback Function to call when FEC data received
 */
void ble_fec_client_register_callback(fec_data_callback_t callback);

/**
 * @brief Register connection callback
 * @param callback Function to call on connect/disconnect
 */
void ble_fec_client_register_conn_callback(fec_conn_callback_t callback);

/**
 * @brief Check if trainer is connected
 * @return true if connected
 */
bool ble_fec_client_is_connected(void);

/**
 * @brief Get current FEC data
 * @param info Pointer to fec_info_t to fill
 * @return APP_OK on success
 */
app_err_t ble_fec_client_get_info(fec_info_t *info);

/**
 * @brief Get full FEC data
 * @param data Pointer to fec_data_t to fill
 * @return APP_OK on success
 */
app_err_t ble_fec_client_get_data(fec_data_t *data);

/**
 * @brief Set simulation grade
 * @param grade Grade in percent (-25 to +25)
 * @return APP_OK on success
 */
app_err_t ble_fec_client_set_grade(int8_t grade);

/**
 * @brief Set target power (ERG mode)
 * @param power Target power in watts
 * @return APP_OK on success
 */
app_err_t ble_fec_client_set_target_power(uint16_t power);

/**
 * @brief Set resistance level
 * @param level Resistance 0-100
 * @return APP_OK on success
 */
app_err_t ble_fec_client_set_resistance(uint8_t level);

/**
 * @brief Set control mode
 * @param mode Control mode
 * @return APP_OK on success
 */
app_err_t ble_fec_client_set_mode(fec_mode_t mode);

/**
 * @brief Start trainer control
 * @return APP_OK on success
 */
app_err_t ble_fec_client_start(void);

/**
 * @brief Stop trainer control
 * @return APP_OK on success
 */
app_err_t ble_fec_client_stop(void);

/**
 * @brief Handle connection event (called by ble_manager)
 * @param conn Connection handle
 */
void ble_fec_client_on_connect(struct bt_conn *conn);

/**
 * @brief Handle disconnection event (called by ble_manager)
 * @param conn Connection handle
 */
void ble_fec_client_on_disconnect(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif /* BLE_FEC_CLIENT_H */
