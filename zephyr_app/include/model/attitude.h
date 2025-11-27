/**
 * @file attitude.h
 * @brief Attitude manager - aggregates all sensor data
 *
 * Combines GPS, barometer, IMU, and other sensor data into
 * a unified state representation.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef MODEL_ATTITUDE_H
#define MODEL_ATTITUDE_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Extended attitude data with computed values */
typedef struct {
    attitude_t base;            /**< Base attitude data */
    float heading;              /**< Compass heading (degrees) */
    float temperature;          /**< Ambient temperature (Celsius) */
    float pressure;             /**< Barometric pressure (Pa) */
    float baro_altitude;        /**< Barometer-derived altitude (m) */
    uint8_t battery_soc;        /**< Battery state of charge (%) */
    float battery_voltage;      /**< Battery voltage (mV) */
    uint8_t hrm_bpm;            /**< Heart rate (bpm) */
    bool hrm_connected;         /**< HRM connection status */
} attitude_ext_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize attitude manager
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_init(void);

/**
 * @brief Update attitude with new GPS data
 * @param loc New location data
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_update_gps(const loc_data_t *loc);

/**
 * @brief Update attitude with new barometer data
 * @param pressure Pressure in Pa
 * @param temperature Temperature in Celsius
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_update_baro(float pressure, float temperature);

/**
 * @brief Update attitude with new IMU data
 * @param heading Compass heading in degrees
 * @param pitch Pitch in degrees
 * @param roll Roll in degrees
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_update_imu(float heading, float pitch, float roll);

/**
 * @brief Update attitude with battery data
 * @param soc State of charge (0-100)
 * @param voltage Voltage in mV
 */
void attitude_update_battery(uint8_t soc, float voltage);

/**
 * @brief Update attitude with heart rate data
 * @param bpm Heart rate in beats per minute
 * @param connected Connection status
 */
void attitude_update_hrm(uint8_t bpm, bool connected);

/**
 * @brief Update attitude with date/time
 * @param date Date data
 */
void attitude_update_datetime(const date_data_t *date);

/**
 * @brief Get current attitude
 * @param att Pointer to store attitude data
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_get(attitude_t *att);

/**
 * @brief Get extended attitude
 * @param att Pointer to store extended attitude data
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_get_ext(attitude_ext_t *att);

/**
 * @brief Get current location
 * @param loc Pointer to store location
 * @return APP_OK on success, error code otherwise
 */
app_err_t attitude_get_location(loc_data_t *loc);

/**
 * @brief Get total distance
 * @return Distance in meters
 */
float attitude_get_distance(void);

/**
 * @brief Get total climb
 * @return Climb in meters
 */
float attitude_get_climb(void);

/**
 * @brief Get elapsed time
 * @return Active time in seconds
 */
uint32_t attitude_get_elapsed_time(void);

/**
 * @brief Get moving time (speed > threshold)
 * @return Moving time in seconds
 */
uint32_t attitude_get_moving_time(void);

/**
 * @brief Get average speed
 * @return Average speed in km/h
 */
float attitude_get_avg_speed(void);

/**
 * @brief Get estimated power
 * @return Power in watts
 */
uint16_t attitude_get_power(void);

/**
 * @brief Set rider weight for power estimation
 *
 * Updates the rider weight used in power calculations.
 * Call this when user settings change.
 *
 * @param weight_kg Rider weight in kilograms
 */
void attitude_set_rider_weight(float weight_kg);

/**
 * @brief Reset attitude state
 */
void attitude_reset(void);

/**
 * @brief Compute derived values (call periodically)
 */
void attitude_compute(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_ATTITUDE_H */
