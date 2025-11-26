/**
 * @file baro.h
 * @brief Barometer driver interface for stravaV10
 *
 * Supports BME280/BMP280 barometric pressure sensor.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_BARO_H
#define DRIVERS_BARO_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Barometer data structure */
typedef struct {
    float pressure;     /**< Pressure in Pa */
    float temperature;  /**< Temperature in Celsius */
    float humidity;     /**< Humidity in % (BME280 only) */
    float altitude;     /**< Calculated altitude in meters */
    uint32_t timestamp; /**< Measurement timestamp */
    bool valid;         /**< Data validity flag */
} baro_data_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize barometer driver
 * @return APP_OK on success, error code otherwise
 */
app_err_t baro_init(void);

/**
 * @brief Trigger a new measurement
 * @return APP_OK on success, error code otherwise
 */
app_err_t baro_trigger(void);

/**
 * @brief Read latest measurement data
 * @param data Pointer to store measurement data
 * @return APP_OK on success, error code otherwise
 */
app_err_t baro_read(baro_data_t *data);

/**
 * @brief Set reference pressure for altitude calculation
 * @param ref_pressure Reference pressure in Pa (sea level)
 */
void baro_set_reference(float ref_pressure);

/**
 * @brief Get current reference pressure
 * @return Reference pressure in Pa
 */
float baro_get_reference(void);

/**
 * @brief Calculate altitude from pressure
 * @param pressure Pressure in Pa
 * @return Altitude in meters
 */
float baro_pressure_to_altitude(float pressure);

/**
 * @brief Put sensor to sleep mode
 * @return APP_OK on success, error code otherwise
 */
app_err_t baro_sleep(void);

/**
 * @brief Wake sensor from sleep
 * @return APP_OK on success, error code otherwise
 */
app_err_t baro_wake(void);

/**
 * @brief Check if new data is available
 * @return true if new data available
 */
bool baro_is_data_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BARO_H */
