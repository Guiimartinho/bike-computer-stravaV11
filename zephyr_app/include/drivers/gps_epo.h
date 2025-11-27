/**
 * @file gps_epo.h
 * @brief GPS EPO (Extended Prediction Orbit) / AGPS module
 *
 * Handles EPO file reading from SD card and transfer to GPS module
 * for faster time-to-first-fix.
 *
 * Based on MediaTek MTK3339 EPO format (MTK14.EPO).
 *
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_GPS_EPO_H
#define DRIVERS_GPS_EPO_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** EPO file name on SD card */
#define EPO_FILENAME            "/SD:/MTK14.EPO"

/** Number of GPS satellites */
#define EPO_SAT_COUNT           32U

/** EPO data size per satellite (bytes) */
#define EPO_SAT_DATA_SIZE       72U

/** Number of satellites per segment */
#define EPO_SEGMENT_SATS        EPO_SAT_COUNT

/** Total data size per 6-hour segment (bytes) */
#define EPO_SEGMENT_SIZE        (EPO_SAT_COUNT * EPO_SAT_DATA_SIZE)

/** Maximum 6-hour segments in EPO file (3.5 days = 14 segments) */
#define EPO_MAX_SEGMENTS        14U

/** EPO file header size (4 bytes: GPS hour) */
#define EPO_HEADER_SIZE         4U

/** Hours per EPO segment */
#define EPO_HOURS_PER_SEGMENT   6U

/** EPO transfer packet size */
#define EPO_PACKET_SIZE         191U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** EPO satellite data packet */
typedef struct {
    uint8_t sat_number;                     /**< Satellite number (1-32) */
    uint8_t sat[EPO_SAT_DATA_SIZE];        /**< Satellite ephemeris data */
} epo_sat_data_t;

/** EPO state enumeration */
typedef enum {
    EPO_STATE_IDLE = 0,     /**< Not running */
    EPO_STATE_CHECKING,     /**< Checking EPO file */
    EPO_STATE_STARTING,     /**< Starting transfer */
    EPO_STATE_RUNNING,      /**< Transfer in progress */
    EPO_STATE_COMPLETE,     /**< Transfer complete */
    EPO_STATE_ERROR         /**< Error occurred */
} epo_state_t;

/** EPO result codes */
typedef enum {
    EPO_RESULT_OK = 0,          /**< Success */
    EPO_RESULT_NO_FILE,         /**< EPO file not found */
    EPO_RESULT_INVALID_FILE,    /**< Invalid/corrupted file */
    EPO_RESULT_EXPIRED,         /**< EPO data expired */
    EPO_RESULT_NO_DATE,         /**< No valid GPS date for segment calc */
    EPO_RESULT_TRANSFER_ERR,    /**< Transfer error */
    EPO_RESULT_BUSY             /**< Already running */
} epo_result_t;

/** EPO status info */
typedef struct {
    epo_state_t state;          /**< Current state */
    epo_result_t last_result;   /**< Last operation result */
    uint16_t file_size;         /**< EPO file size in bytes */
    uint8_t segments_available; /**< Available 6-hour segments */
    uint8_t current_segment;    /**< Current segment being processed */
    uint8_t sats_transferred;   /**< Satellites transferred in current segment */
    uint32_t last_update_time;  /**< Last successful transfer timestamp */
} epo_status_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize EPO module
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_epo_init(void);

/**
 * @brief Check if EPO file exists and is valid
 * @param current_gps_hour Current GPS hour (hours since GPS epoch)
 * @return EPO_RESULT_OK if valid, error code otherwise
 */
epo_result_t gps_epo_check_file(uint32_t current_gps_hour);

/**
 * @brief Get EPO file size
 * @return File size in bytes, or -1 if error
 */
int32_t gps_epo_get_file_size(void);

/**
 * @brief Start EPO transfer to GPS module
 * @param current_gps_hour Current GPS hour for segment selection
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_epo_start_transfer(uint32_t current_gps_hour);

/**
 * @brief Process EPO transfer state machine
 *
 * Call this periodically from main loop or GPS task.
 * Returns when all data transferred or error occurs.
 */
void gps_epo_process(void);

/**
 * @brief Stop EPO transfer
 * @param delete_file If true, delete EPO file after stop
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_epo_stop(bool delete_file);

/**
 * @brief Get current EPO status
 * @param status Pointer to store status info
 * @return APP_OK on success, error code otherwise
 */
app_err_t gps_epo_get_status(epo_status_t *status);

/**
 * @brief Check if EPO transfer is in progress
 * @return true if transfer is running
 */
bool gps_epo_is_busy(void);

/**
 * @brief Calculate GPS hour from date/time
 * @param date Date structure
 * @return GPS hour (hours since Jan 6, 1980)
 */
uint32_t gps_epo_calc_gps_hour(const date_data_t *date);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_GPS_EPO_H */
