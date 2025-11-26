/**
 * @file stc3100.h
 * @brief STC3100 Battery Fuel Gauge driver for stravaV10
 *
 * Driver for ST STC3100 battery monitoring IC.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_STC3100_H
#define DRIVERS_STC3100_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Battery data structure */
typedef struct {
    float voltage;      /**< Battery voltage in mV */
    float current;      /**< Battery current in mA (negative = discharge) */
    float temperature;  /**< Battery temperature in Celsius */
    float charge;       /**< Accumulated charge in mAh */
    uint8_t soc;        /**< State of charge in percent (0-100) */
    uint32_t timestamp; /**< Measurement timestamp */
    bool valid;         /**< Data validity flag */
} battery_data_t;

/** Battery configuration */
typedef struct {
    uint32_t rsense;        /**< Sense resistor value in mOhm */
    uint32_t capacity_mah;  /**< Battery capacity in mAh */
    uint16_t voltage_min;   /**< Minimum voltage in mV (0% SoC) */
    uint16_t voltage_max;   /**< Maximum voltage in mV (100% SoC) */
} battery_config_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize STC3100 driver
 * @param config Battery configuration
 * @return APP_OK on success, error code otherwise
 */
app_err_t stc3100_init(const battery_config_t *config);

/**
 * @brief Trigger a new measurement
 * @return APP_OK on success, error code otherwise
 */
app_err_t stc3100_trigger(void);

/**
 * @brief Read latest battery data
 * @param data Pointer to store battery data
 * @return APP_OK on success, error code otherwise
 */
app_err_t stc3100_read(battery_data_t *data);

/**
 * @brief Get battery voltage
 * @return Voltage in mV
 */
float stc3100_get_voltage(void);

/**
 * @brief Get battery current
 * @return Current in mA (negative = discharge)
 */
float stc3100_get_current(void);

/**
 * @brief Get battery temperature
 * @return Temperature in Celsius
 */
float stc3100_get_temperature(void);

/**
 * @brief Get state of charge
 * @return SoC in percent (0-100)
 */
uint8_t stc3100_get_soc(void);

/**
 * @brief Get accumulated charge
 * @return Charge in mAh
 */
float stc3100_get_charge(void);

/**
 * @brief Reset charge counter
 */
void stc3100_reset_charge(void);

/**
 * @brief Get average current over time
 * @return Average current in mA
 */
float stc3100_get_avg_current(void);

/**
 * @brief Put fuel gauge to sleep
 * @return APP_OK on success, error code otherwise
 */
app_err_t stc3100_sleep(void);

/**
 * @brief Wake fuel gauge from sleep
 * @return APP_OK on success, error code otherwise
 */
app_err_t stc3100_wake(void);

/**
 * @brief Reset the fuel gauge
 * @return APP_OK on success, error code otherwise
 */
app_err_t stc3100_reset(void);

/**
 * @brief Check if new data is available
 * @return true if new data available
 */
bool stc3100_is_data_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_STC3100_H */
