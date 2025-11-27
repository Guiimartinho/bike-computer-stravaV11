/**
 * @file gps_epo.c
 * @brief GPS EPO (Extended Prediction Orbit) module implementation
 *
 * Handles EPO file reading from SD card and transfer to GPS module.
 * Based on MediaTek MTK3339 protocol.
 *
 * Follows MISRA C:2012 guidelines.
 */

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "drivers/gps_epo.h"
#include "hal/hal_uart.h"

LOG_MODULE_REGISTER(gps_epo, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** PMTK command buffer size */
#define PMTK_CMD_SIZE       256U

/** GPS UART port */
#define GPS_UART_PORT       HAL_UART_GPS

/** EPO packet delay between transfers (ms) */
#define EPO_PACKET_DELAY_MS 50U

/** GPS epoch: January 6, 1980 */
#define GPS_EPOCH_YEAR      1980U
#define GPS_EPOCH_MONTH     1U
#define GPS_EPOCH_DAY       6U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Module initialized flag */
static bool initialized;

/** EPO file object */
static struct fs_file_t epo_file;

/** Current EPO status */
static epo_status_t status;

/** Current satellite index */
static uint8_t current_sat_idx;

/** Read buffer */
static uint8_t read_buffer[EPO_SAT_DATA_SIZE + 4U];

/** PMTK command buffer */
static char pmtk_cmd[PMTK_CMD_SIZE];

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate PMTK checksum
 * @param str PMTK string (without $, * and checksum)
 * @return Checksum byte
 */
static uint8_t calc_pmtk_checksum(const char *str)
{
    uint8_t checksum = 0U;
    while (*str != '\0') {
        checksum ^= (uint8_t)*str;
        str++;
    }
    return checksum;
}

/**
 * @brief Send PMTK command to GPS
 * @param cmd Command string (without $ and checksum)
 * @return APP_OK on success
 */
static app_err_t send_pmtk_cmd(const char *cmd)
{
    if (cmd == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    uint8_t checksum = calc_pmtk_checksum(cmd);

    int len = snprintf(pmtk_cmd, sizeof(pmtk_cmd),
                       "$%s*%02X\r\n", cmd, checksum);

    if (len <= 0) {
        return APP_ERR_INTERNAL;
    }

    /* Send via GPS UART */
    app_err_t err = hal_uart_transmit(GPS_UART_PORT,
                                      (const uint8_t *)pmtk_cmd,
                                      (size_t)len);

    LOG_DBG("PMTK: %s", pmtk_cmd);

    return err;
}

/**
 * @brief Send EPO binary packet to GPS
 *
 * Uses PMTK721 command to transfer satellite ephemeris data.
 *
 * @param sat_num Satellite number (1-32)
 * @param data Satellite data (72 bytes)
 * @return APP_OK on success
 */
static app_err_t send_epo_packet(uint8_t sat_num, const uint8_t *data)
{
    if ((data == NULL) || (sat_num == 0U) || (sat_num > EPO_SAT_COUNT)) {
        return APP_ERR_INVALID_PARAM;
    }

    /* PMTK721 format: $PMTK721,sat_num,data_hex*checksum
     * Each satellite data is 72 bytes in hex = 144 chars
     */

    int pos = snprintf(pmtk_cmd, sizeof(pmtk_cmd), "PMTK721,%u,", sat_num);

    /* Convert data to hex string */
    for (uint8_t i = 0U; i < EPO_SAT_DATA_SIZE; i++) {
        pos += snprintf(&pmtk_cmd[pos], sizeof(pmtk_cmd) - (size_t)pos,
                        "%02X", data[i]);
        if ((size_t)pos >= sizeof(pmtk_cmd) - 8U) {
            break;
        }
    }

    return send_pmtk_cmd(pmtk_cmd);
}

/**
 * @brief Enable EPO function on GPS module
 * @return APP_OK on success
 */
static app_err_t enable_epo_function(void)
{
    /* PMTK253,1 enables EPO function */
    return send_pmtk_cmd("PMTK253,1");
}

/**
 * @brief Clear EPO data on GPS module
 * @return APP_OK on success
 */
static app_err_t clear_epo_data(void)
{
    /* PMTK127 clears EPO data */
    return send_pmtk_cmd("PMTK127");
}

/**
 * @brief Calculate days since GPS epoch
 */
static uint32_t days_since_gps_epoch(uint16_t year, uint8_t month, uint8_t day)
{
    /* Simplified calculation - not accounting for all edge cases */
    static const uint16_t days_in_month[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    uint32_t days = 0U;

    /* Years since 1980 */
    for (uint16_t y = GPS_EPOCH_YEAR; y < year; y++) {
        days += 365U;
        /* Add leap day */
        if (((y % 4U) == 0U) && (((y % 100U) != 0U) || ((y % 400U) == 0U))) {
            days += 1U;
        }
    }

    /* Months in current year */
    if (month > 0U && month <= 12U) {
        days += days_in_month[month - 1U];
        /* Leap year adjustment */
        if ((month > 2U) &&
            ((year % 4U) == 0U) &&
            (((year % 100U) != 0U) || ((year % 400U) == 0U))) {
            days += 1U;
        }
    }

    /* Days */
    days += (uint32_t)day;

    /* Subtract GPS epoch start (Jan 6, 1980 = day 6) */
    if (days >= 6U) {
        days -= 6U;
    }

    return days;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t gps_epo_init(void)
{
    if (initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    (void)memset(&status, 0, sizeof(status));
    fs_file_t_init(&epo_file);
    current_sat_idx = 0U;

    initialized = true;
    LOG_INF("GPS EPO module initialized");

    return APP_OK;
}

epo_result_t gps_epo_check_file(uint32_t current_gps_hour)
{
    struct fs_dirent entry;
    int err;

    /* Check if file exists */
    err = fs_stat(EPO_FILENAME, &entry);
    if (err != 0) {
        LOG_WRN("EPO file not found: %s", EPO_FILENAME);
        return EPO_RESULT_NO_FILE;
    }

    /* Check minimum size */
    if (entry.size < (EPO_HEADER_SIZE + EPO_SEGMENT_SIZE)) {
        LOG_WRN("EPO file too small: %zu bytes", entry.size);
        return EPO_RESULT_INVALID_FILE;
    }

    status.file_size = (uint16_t)entry.size;
    status.segments_available = (uint8_t)((entry.size - EPO_HEADER_SIZE) /
                                          EPO_SEGMENT_SIZE);

    if (status.segments_available > EPO_MAX_SEGMENTS) {
        status.segments_available = EPO_MAX_SEGMENTS;
    }

    /* Open file to read header and check date */
    err = fs_open(&epo_file, EPO_FILENAME, FS_O_READ);
    if (err != 0) {
        LOG_ERR("Failed to open EPO file: %d", err);
        return EPO_RESULT_INVALID_FILE;
    }

    /* Read header (GPS hour at file creation) */
    uint32_t file_gps_hour = 0U;
    ssize_t bytes = fs_read(&epo_file, &file_gps_hour, sizeof(file_gps_hour));
    (void)fs_close(&epo_file);

    if (bytes != sizeof(file_gps_hour)) {
        LOG_ERR("Failed to read EPO header");
        return EPO_RESULT_INVALID_FILE;
    }

    /* Mask to 24-bit value */
    file_gps_hour &= 0x00FFFFFFU;

    LOG_INF("EPO file: %u bytes, %u segments, start hour %u",
            status.file_size, status.segments_available,
            (unsigned)file_gps_hour);

    /* Check if current time is within EPO validity */
    if (current_gps_hour > 0U) {
        int32_t segment = (int32_t)(current_gps_hour - file_gps_hour) /
                          (int32_t)EPO_HOURS_PER_SEGMENT;

        if ((segment < 0) || (segment >= (int32_t)status.segments_available)) {
            LOG_WRN("EPO data expired (segment %d, available %u)",
                    (int)segment, status.segments_available);
            return EPO_RESULT_EXPIRED;
        }

        LOG_INF("EPO segment %d valid", (int)segment);
    }

    return EPO_RESULT_OK;
}

int32_t gps_epo_get_file_size(void)
{
    struct fs_dirent entry;

    if (fs_stat(EPO_FILENAME, &entry) != 0) {
        return -1;
    }

    return (int32_t)entry.size;
}

app_err_t gps_epo_start_transfer(uint32_t current_gps_hour)
{
    if (!initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (status.state == EPO_STATE_RUNNING) {
        return APP_ERR_BUSY;
    }

    /* Check file first */
    epo_result_t result = gps_epo_check_file(current_gps_hour);
    if (result != EPO_RESULT_OK) {
        status.last_result = result;
        status.state = EPO_STATE_ERROR;
        return APP_ERR_IO;
    }

    /* Calculate correct segment */
    int err = fs_open(&epo_file, EPO_FILENAME, FS_O_READ);
    if (err != 0) {
        status.last_result = EPO_RESULT_INVALID_FILE;
        status.state = EPO_STATE_ERROR;
        return APP_ERR_IO;
    }

    /* Read header to get file GPS hour */
    uint32_t file_gps_hour = 0U;
    (void)fs_read(&epo_file, &file_gps_hour, sizeof(file_gps_hour));
    file_gps_hour &= 0x00FFFFFFU;

    /* Calculate segment index */
    uint32_t segment = 0U;
    if (current_gps_hour > file_gps_hour) {
        segment = (current_gps_hour - file_gps_hour) / EPO_HOURS_PER_SEGMENT;
    }

    if (segment >= status.segments_available) {
        segment = 0U; /* Fallback to first segment */
    }

    /* Seek to correct segment */
    off_t offset = (off_t)(EPO_HEADER_SIZE + (segment * EPO_SEGMENT_SIZE));
    err = fs_seek(&epo_file, offset, FS_SEEK_SET);
    if (err != 0) {
        (void)fs_close(&epo_file);
        status.last_result = EPO_RESULT_INVALID_FILE;
        status.state = EPO_STATE_ERROR;
        return APP_ERR_IO;
    }

    /* Enable EPO on GPS module */
    (void)enable_epo_function();
    k_msleep(100);

    /* Clear existing EPO data */
    (void)clear_epo_data();
    k_msleep(100);

    status.current_segment = (uint8_t)segment;
    status.sats_transferred = 0U;
    status.state = EPO_STATE_RUNNING;
    status.last_result = EPO_RESULT_OK;
    current_sat_idx = 0U;

    LOG_INF("EPO transfer started (segment %u)", (unsigned)segment);

    return APP_OK;
}

void gps_epo_process(void)
{
    if (!initialized || (status.state != EPO_STATE_RUNNING)) {
        return;
    }

    /* Read next satellite data */
    if (current_sat_idx >= EPO_SAT_COUNT) {
        /* All satellites transferred */
        status.state = EPO_STATE_COMPLETE;
        status.last_result = EPO_RESULT_OK;
        status.last_update_time = k_uptime_get_32();
        (void)fs_close(&epo_file);
        LOG_INF("EPO transfer complete (%u satellites)", current_sat_idx);
        return;
    }

    /* Read satellite data */
    ssize_t bytes = fs_read(&epo_file, read_buffer, EPO_SAT_DATA_SIZE);
    if (bytes != EPO_SAT_DATA_SIZE) {
        LOG_WRN("EPO read error at sat %u", current_sat_idx);
        /* Try to continue with next satellite */
        if (bytes <= 0) {
            status.state = EPO_STATE_ERROR;
            status.last_result = EPO_RESULT_TRANSFER_ERR;
            (void)fs_close(&epo_file);
            return;
        }
    }

    /* Send to GPS module */
    app_err_t err = send_epo_packet(current_sat_idx + 1U, read_buffer);
    if (err != APP_OK) {
        LOG_WRN("EPO send error at sat %u", current_sat_idx);
    }

    current_sat_idx++;
    status.sats_transferred = current_sat_idx;

    /* Small delay between packets */
    k_msleep(EPO_PACKET_DELAY_MS);

    LOG_DBG("EPO sat %u/%u transferred", current_sat_idx, EPO_SAT_COUNT);
}

app_err_t gps_epo_stop(bool delete_file)
{
    if (!initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (status.state == EPO_STATE_RUNNING) {
        (void)fs_close(&epo_file);
    }

    status.state = EPO_STATE_IDLE;

    if (delete_file) {
        int err = fs_unlink(EPO_FILENAME);
        if (err != 0) {
            LOG_WRN("Failed to delete EPO file: %d", err);
        } else {
            LOG_INF("EPO file deleted");
        }
    }

    return APP_OK;
}

app_err_t gps_epo_get_status(epo_status_t *out_status)
{
    if (out_status == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *out_status = status;
    return APP_OK;
}

bool gps_epo_is_busy(void)
{
    return (status.state == EPO_STATE_RUNNING);
}

uint32_t gps_epo_calc_gps_hour(const date_data_t *date)
{
    if (date == NULL) {
        return 0U;
    }

    /* Parse date (DDMMYY format) */
    uint8_t day = (uint8_t)(date->date / 10000U);
    uint8_t month = (uint8_t)((date->date / 100U) % 100U);
    uint16_t year = (uint16_t)(date->date % 100U);

    /* Adjust for 2000+ years */
    if (year < 80U) {
        year += 2000U;
    } else {
        year += 1900U;
    }

    /* Calculate days since GPS epoch */
    uint32_t days = days_since_gps_epoch(year, month, day);

    /* Convert to hours and add time of day */
    uint32_t hours = days * 24U;
    hours += date->secj / 3600U;

    return hours;
}
