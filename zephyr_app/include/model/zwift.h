/**
 * @file zwift.h
 * @brief Zwift indoor simulation mode
 *
 * Provides indoor trainer simulation mode that receives position data
 * from Zwift via BLE NUS instead of GPS.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef MODEL_ZWIFT_H
#define MODEL_ZWIFT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Zwift simulation data received via BLE */
typedef struct {
    float lat;              /**< Simulated latitude */
    float lon;              /**< Simulated longitude */
    float alt;              /**< Simulated altitude (m) */
    float speed;            /**< Speed (km/h) */
    float distance;         /**< Total distance (m) */
    float gradient;         /**< Road gradient (%) */
    uint16_t power;         /**< Power (watts) */
    uint32_t timestamp;     /**< Last update timestamp */
    bool valid;             /**< Data validity flag */
} zwift_data_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize Zwift mode
 * @return APP_OK on success
 */
app_err_t zwift_init(void);

/**
 * @brief Enter Zwift mode
 * @note This will put GPS to standby and start listening for BLE data
 * @return APP_OK on success
 */
app_err_t zwift_enter(void);

/**
 * @brief Exit Zwift mode
 * @return APP_OK on success
 */
app_err_t zwift_exit(void);

/**
 * @brief Check if Zwift mode is active
 * @return true if in Zwift mode
 */
bool zwift_is_active(void);

/**
 * @brief Update Zwift data from BLE NUS
 * @param data Raw data buffer from BLE
 * @param len Length of data
 */
void zwift_update_from_ble(const uint8_t *data, uint16_t len);

/**
 * @brief Get current Zwift simulation data
 * @param data Pointer to store data
 * @return APP_OK on success
 */
app_err_t zwift_get_data(zwift_data_t *data);

/**
 * @brief Process Zwift simulation (call periodically)
 */
void zwift_process(void);

/**
 * @brief Set FEC trainer control from Zwift
 * @param gradient Target gradient in percent
 */
void zwift_set_gradient(float gradient);

/**
 * @brief Set FEC trainer target power from Zwift
 * @param power Target power in watts
 */
void zwift_set_target_power(uint16_t power);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_ZWIFT_H */
