/**
 * @file gps_mgmt.h
 * @brief GPS Management module for stravaV10
 *
 * Handles GPS module communication, NMEA parsing, and power management.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_GPS_MGMT_H
#define DRIVERS_GPS_MGMT_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** GPS power state */
typedef enum {
    GPS_STATE_OFF = 0,      /**< GPS module off */
    GPS_STATE_INIT,         /**< Initializing */
    GPS_STATE_ACQUIRING,    /**< Acquiring fix */
    GPS_STATE_FIX_2D,       /**< 2D fix (no altitude) */
    GPS_STATE_FIX_3D,       /**< 3D fix */
    GPS_STATE_STANDBY       /**< Standby/sleep mode */
} gps_state_t;

/** GPS fix quality */
typedef enum {
    GPS_FIX_INVALID = 0,    /**< No fix */
    GPS_FIX_GPS,            /**< GPS fix */
    GPS_FIX_DGPS,           /**< Differential GPS fix */
    GPS_FIX_PPS,            /**< PPS fix */
    GPS_FIX_RTK,            /**< Real Time Kinematic */
    GPS_FIX_FLOAT_RTK,      /**< Float RTK */
    GPS_FIX_ESTIMATED,      /**< Estimated (dead reckoning) */
    GPS_FIX_MANUAL,         /**< Manual input mode */
    GPS_FIX_SIMULATION      /**< Simulation mode */
} gps_fix_quality_t;

/** GPS data structure */
typedef struct {
    loc_data_t location;        /**< Position data */
    date_data_t datetime;       /**< Date/time data */
    gps_fix_quality_t quality;  /**< Fix quality */
    uint8_t satellites;         /**< Number of satellites used */
    float hdop;                 /**< Horizontal dilution of precision */
    float vdop;                 /**< Vertical dilution of precision */
    float pdop;                 /**< Position dilution of precision */
    bool fix_valid;             /**< Fix validity flag */
} gps_data_t;

/** GPS callback for new fix */
typedef void (*gps_fix_callback_t)(const gps_data_t *data);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize GPS management module
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_init(void);

/**
 * @brief Start GPS acquisition
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_start(void);

/**
 * @brief Stop GPS acquisition
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_stop(void);

/**
 * @brief Put GPS in standby mode
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_standby(void);

/**
 * @brief Wake GPS from standby
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_wake(void);

/**
 * @brief Reset GPS module
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_reset(void);

/**
 * @brief Get current GPS state
 * @return Current GPS state
 */
gps_state_t gps_mgmt_get_state(void);

/**
 * @brief Check if GPS has valid fix
 * @return true if fix is valid
 */
bool gps_mgmt_has_fix(void);

/**
 * @brief Get latest GPS data
 * @param data Pointer to store GPS data
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_get_data(gps_data_t *data);

/**
 * @brief Get latest location data only
 * @param loc Pointer to store location data
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_get_location(loc_data_t *loc);

/**
 * @brief Register callback for new GPS fix
 * @param callback Function to call on new fix
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_register_callback(gps_fix_callback_t callback);

/**
 * @brief Set GPS update rate
 * @param rate_hz Update rate in Hz (1-10)
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_mgmt_set_rate(uint8_t rate_hz);

/**
 * @brief GPS processing task (call from main loop or thread)
 */
void gps_mgmt_process(void);

/**
 * @brief Get number of satellites in view
 * @return Number of satellites
 */
uint8_t gps_mgmt_get_satellites(void);

/**
 * @brief Check if GPS fix indicator pin is active
 * @return true if fix indicator is active
 */
bool gps_mgmt_check_fix_pin(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_GPS_MGMT_H */
