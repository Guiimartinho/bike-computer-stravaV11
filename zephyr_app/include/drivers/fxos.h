/**
 * @file fxos.h
 * @brief FXOS8700 Accelerometer/Magnetometer driver for stravaV10
 *
 * Driver for NXP FXOS8700 6-axis sensor.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_FXOS_H
#define DRIVERS_FXOS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Accelerometer data structure (raw values in mg) */
typedef struct {
    int16_t x;          /**< X-axis acceleration */
    int16_t y;          /**< Y-axis acceleration */
    int16_t z;          /**< Z-axis acceleration */
} fxos_accel_t;

/** Magnetometer data structure (raw values in uT) */
typedef struct {
    int16_t x;          /**< X-axis magnetic field */
    int16_t y;          /**< Y-axis magnetic field */
    int16_t z;          /**< Z-axis magnetic field */
} fxos_mag_t;

/** Combined sensor data */
typedef struct {
    fxos_accel_t accel;     /**< Accelerometer data */
    fxos_mag_t mag;         /**< Magnetometer data */
    float yaw;              /**< Calculated yaw angle (radians) */
    float pitch;            /**< Calculated pitch angle (radians) */
    float roll;             /**< Calculated roll angle (radians) */
    uint32_t timestamp;     /**< Measurement timestamp */
    bool valid;             /**< Data validity flag */
} fxos_data_t;

/** Calibration data */
typedef struct {
    int16_t mag_offset_x;   /**< Magnetometer X offset */
    int16_t mag_offset_y;   /**< Magnetometer Y offset */
    int16_t mag_offset_z;   /**< Magnetometer Z offset */
    float mag_scale_x;      /**< Magnetometer X scale */
    float mag_scale_y;      /**< Magnetometer Y scale */
    float mag_scale_z;      /**< Magnetometer Z scale */
    bool calibrated;        /**< Calibration valid flag */
} fxos_calib_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize FXOS8700 driver
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_init(void);

/**
 * @brief Trigger a new measurement
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_trigger(void);

/**
 * @brief Read latest measurement data
 * @param data Pointer to store measurement data
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_read(fxos_data_t *data);

/**
 * @brief Start magnetometer calibration
 *
 * Calibration requires rotating the device in all directions.
 */
void fxos_calibration_start(void);

/**
 * @brief Stop magnetometer calibration and compute offsets
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_calibration_stop(void);

/**
 * @brief Get calibration data
 * @param calib Pointer to store calibration data
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_get_calibration(fxos_calib_t *calib);

/**
 * @brief Set calibration data
 * @param calib Pointer to calibration data
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_set_calibration(const fxos_calib_t *calib);

/**
 * @brief Get yaw angle (heading)
 * @param yaw Pointer to store yaw in radians
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_get_yaw(float *yaw);

/**
 * @brief Get pitch angle
 * @param pitch Pointer to store pitch in radians
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_get_pitch(float *pitch);

/**
 * @brief Get roll angle
 * @param roll Pointer to store roll in radians
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_get_roll(float *roll);

/**
 * @brief Put sensor to sleep mode
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_sleep(void);

/**
 * @brief Wake sensor from sleep
 * @return APP_OK on success, error code otherwise
 */
app_err_t fxos_wake(void);

/**
 * @brief Periodic processing (call from main loop)
 */
void fxos_process(void);

/**
 * @brief Check if new data is available
 * @return true if new data available
 */
bool fxos_is_data_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_FXOS_H */
