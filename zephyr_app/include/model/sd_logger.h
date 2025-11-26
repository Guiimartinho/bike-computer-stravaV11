/**
 * @file sd_logger.h
 * @brief SD Card position and sensor logging
 *
 * Implements buffered logging of position and sensor data to SD card.
 * Based on original Attitude.cpp sd_save_pos_buffer() implementation.
 *
 * Data is buffered in RAM and written to SD when buffer is full or
 * at periodic intervals to minimize SD card wear and power consumption.
 */

#ifndef MODEL_SD_LOGGER_H
#define MODEL_SD_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Number of entries in position buffer (from original ATT_BUFFER_NB_ELEM) */
#define SD_LOG_BUFFER_SIZE      5U

/** Minimum distance between logged points (meters) */
#define SD_LOG_MIN_DISTANCE_M   15.0f

/** Maximum log file size before rotation (bytes) */
#define SD_LOG_MAX_FILE_SIZE    (10U * 1024U * 1024U)  /* 10MB */

/** Log file directory */
#define SD_LOG_DIRECTORY        "/SD:/logs"

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Sensor snapshot data
 */
typedef struct {
    uint16_t power;         /**< Instantaneous power (W) */
    uint8_t bpm;            /**< Heart rate (bpm) */
    uint8_t cadence;        /**< Cadence (rpm) */
    uint16_t speed;         /**< Speed (0.01 km/h) */
} sd_log_sensors_t;

/**
 * @brief Altitude data snapshot
 */
typedef struct {
    float baro_alt;         /**< Raw barometer altitude (m) */
    float filt_alt;         /**< Filtered altitude (m) */
    float gps_alt;          /**< GPS altitude (m) */
    int8_t slope;           /**< Slope (%) */
} sd_log_altitude_t;

/**
 * @brief Complete log entry (position + sensors + altitude)
 */
typedef struct {
    loc_data_t loc;             /**< Position data */
    date_data_t date;           /**< Date/time data */
    sd_log_sensors_t sensors;   /**< Sensor data */
    sd_log_altitude_t alti;     /**< Altitude data */
    float distance;             /**< Total distance at this point */
    float climb;                /**< Total climb at this point */
    uint32_t timestamp;         /**< System timestamp */
} sd_log_entry_t;

/**
 * @brief Logger state
 */
typedef struct {
    sd_log_entry_t buffer[SD_LOG_BUFFER_SIZE];  /**< Entry buffer */
    uint8_t buffer_count;                        /**< Entries in buffer */
    float last_log_distance;                     /**< Distance at last log */
    uint32_t total_entries;                      /**< Total entries written */
    uint32_t file_size;                          /**< Current file size */
    char filename[64];                           /**< Current log filename */
    bool file_open;                              /**< File is open */
    bool is_initialized;                         /**< Initialization flag */
} sd_logger_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize SD logger
 * @param logger Pointer to logger state
 * @return APP_OK on success
 */
app_err_t sd_logger_init(sd_logger_t *logger);

/**
 * @brief Start logging session
 *
 * Creates new log file with timestamp-based name.
 *
 * @param logger Pointer to logger state
 * @param date Current date for filename
 * @return APP_OK on success
 */
app_err_t sd_logger_start(sd_logger_t *logger, const date_data_t *date);

/**
 * @brief Stop logging session
 *
 * Flushes remaining buffer and closes file.
 *
 * @param logger Pointer to logger state
 * @return APP_OK on success
 */
app_err_t sd_logger_stop(sd_logger_t *logger);

/**
 * @brief Add log entry
 *
 * Entry is buffered and written when buffer is full.
 * Entries are only added if distance threshold is met.
 *
 * @param logger Pointer to logger state
 * @param entry Log entry data
 * @param current_distance Current total distance
 * @return APP_OK on success
 */
app_err_t sd_logger_add_entry(sd_logger_t *logger,
                              const sd_log_entry_t *entry,
                              float current_distance);

/**
 * @brief Force flush buffer to SD card
 * @param logger Pointer to logger state
 * @return APP_OK on success
 */
app_err_t sd_logger_flush(sd_logger_t *logger);

/**
 * @brief Get total entries logged
 * @param logger Pointer to logger state
 * @return Total entry count
 */
uint32_t sd_logger_get_count(const sd_logger_t *logger);

/**
 * @brief Check if logger is active
 * @param logger Pointer to logger state
 * @return true if logging
 */
bool sd_logger_is_active(const sd_logger_t *logger);

/**
 * @brief Get current log filename
 * @param logger Pointer to logger state
 * @return Filename string or NULL
 */
const char *sd_logger_get_filename(const sd_logger_t *logger);

/**
 * @brief Create helper to build log entry
 * @param entry Output entry
 * @param loc Position data
 * @param date Date/time data
 * @param power Power in watts
 * @param bpm Heart rate
 * @param cadence Cadence
 * @param speed Speed in 0.01 km/h
 * @param baro_alt Barometer altitude
 * @param filt_alt Filtered altitude
 * @param slope Slope percentage
 * @param distance Total distance
 * @param climb Total climb
 */
void sd_logger_build_entry(sd_log_entry_t *entry,
                           const loc_data_t *loc,
                           const date_data_t *date,
                           uint16_t power,
                           uint8_t bpm,
                           uint8_t cadence,
                           uint16_t speed,
                           float baro_alt,
                           float filt_alt,
                           int8_t slope,
                           float distance,
                           float climb);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_SD_LOGGER_H */
