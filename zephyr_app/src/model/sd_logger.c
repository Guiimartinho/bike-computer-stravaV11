/**
 * @file sd_logger.c
 * @brief SD Card position and sensor logging implementation
 *
 * Based on original Attitude.cpp sd_save_pos_buffer() implementation.
 *
 * Log file format (CSV):
 * timestamp,lat,lon,alt_gps,alt_baro,alt_filt,speed,power,bpm,cadence,slope,dist,climb
 */

#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>

#include "model/sd_logger.h"

LOG_MODULE_REGISTER(sd_logger, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** CSV header line */
static const char csv_header[] =
    "timestamp,lat,lon,alt_gps,alt_baro,alt_filt,speed_kmh,power_w,"
    "bpm,cadence,slope_pct,dist_m,climb_m\r\n";

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Generate log filename from date
 */
static void generate_filename(char *filename, size_t size, const date_data_t *date)
{
    if (date == NULL) {
        /* Use timestamp-based name */
        uint32_t ts = k_uptime_get_32() / 1000U;
        (void)snprintf(filename, size, "%s/log_%08X.csv",
                       SD_LOG_DIRECTORY, (unsigned)ts);
    } else {
        /* Use date-based name (YYMMDD_HHMMSS) */
        uint32_t day = date->date / 10000U;
        uint32_t month = (date->date / 100U) % 100U;
        uint32_t year = date->date % 100U;
        uint32_t hours = date->secj / 3600U;
        uint32_t minutes = (date->secj / 60U) % 60U;
        uint32_t seconds = date->secj % 60U;

        (void)snprintf(filename, size, "%s/%02u%02u%02u_%02u%02u%02u.csv",
                       SD_LOG_DIRECTORY,
                       (unsigned)year, (unsigned)month, (unsigned)day,
                       (unsigned)hours, (unsigned)minutes, (unsigned)seconds);
    }
}

/**
 * @brief Write buffer to file
 */
static app_err_t write_buffer_to_file(sd_logger_t *logger)
{
    struct fs_file_t file;
    int ret;

    if (logger->buffer_count == 0U) {
        return APP_OK;
    }

    fs_file_t_init(&file);

    /* Open or create file */
    ret = fs_open(&file, logger->filename,
                  FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret < 0) {
        LOG_ERR("Failed to open log file: %d", ret);
        return APP_ERR_IO;
    }

    /* Check if we need to write header (new file) */
    struct fs_dirent entry;
    if ((fs_stat(logger->filename, &entry) == 0) && (entry.size == 0)) {
        ret = fs_write(&file, csv_header, strlen(csv_header));
        if (ret < 0) {
            LOG_ERR("Failed to write header: %d", ret);
            (void)fs_close(&file);
            return APP_ERR_IO;
        }
        logger->file_size += (uint32_t)ret;
    }

    /* Write each entry */
    char line[256];

    for (uint8_t i = 0U; i < logger->buffer_count; i++) {
        const sd_log_entry_t *e = &logger->buffer[i];

        int len = snprintf(line, sizeof(line),
                           "%u,%.6f,%.6f,%.1f,%.1f,%.1f,%.2f,%u,%u,%u,%d,%.1f,%.1f\r\n",
                           (unsigned)e->timestamp,
                           (double)e->loc.lat,
                           (double)e->loc.lon,
                           (double)e->alti.gps_alt,
                           (double)e->alti.baro_alt,
                           (double)e->alti.filt_alt,
                           (double)e->sensors.speed / 100.0,
                           (unsigned)e->sensors.power,
                           (unsigned)e->sensors.bpm,
                           (unsigned)e->sensors.cadence,
                           (int)e->alti.slope,
                           (double)e->distance,
                           (double)e->climb);

        if (len > 0) {
            ret = fs_write(&file, line, (size_t)len);
            if (ret < 0) {
                LOG_ERR("Failed to write entry: %d", ret);
                (void)fs_close(&file);
                return APP_ERR_IO;
            }
            logger->file_size += (uint32_t)ret;
            logger->total_entries++;
        }
    }

    (void)fs_close(&file);
    logger->buffer_count = 0U;

    LOG_DBG("Wrote %u entries to %s", logger->buffer_count, logger->filename);

    return APP_OK;
}

/**
 * @brief Ensure log directory exists
 */
static void ensure_directory(void)
{
    struct fs_dirent entry;

    if (fs_stat(SD_LOG_DIRECTORY, &entry) != 0) {
        int ret = fs_mkdir(SD_LOG_DIRECTORY);
        if (ret < 0) {
            LOG_WRN("Failed to create log directory: %d", ret);
        }
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t sd_logger_init(sd_logger_t *logger)
{
    if (logger == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    (void)memset(logger, 0, sizeof(sd_logger_t));
    logger->is_initialized = true;

    LOG_INF("SD logger initialized (buffer=%u entries)", SD_LOG_BUFFER_SIZE);

    return APP_OK;
}

app_err_t sd_logger_start(sd_logger_t *logger, const date_data_t *date)
{
    if ((logger == NULL) || !logger->is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Ensure directory exists */
    ensure_directory();

    /* Generate filename */
    generate_filename(logger->filename, sizeof(logger->filename), date);

    /* Reset state */
    logger->buffer_count = 0U;
    logger->total_entries = 0U;
    logger->file_size = 0U;
    logger->last_log_distance = 0.0f;
    logger->file_open = true;

    LOG_INF("Started logging to %s", logger->filename);

    return APP_OK;
}

app_err_t sd_logger_stop(sd_logger_t *logger)
{
    if ((logger == NULL) || !logger->is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Flush remaining buffer */
    if (logger->buffer_count > 0U) {
        (void)write_buffer_to_file(logger);
    }

    logger->file_open = false;

    LOG_INF("Stopped logging. Total entries: %u", (unsigned)logger->total_entries);

    return APP_OK;
}

app_err_t sd_logger_add_entry(sd_logger_t *logger,
                              const sd_log_entry_t *entry,
                              float current_distance)
{
    if ((logger == NULL) || !logger->is_initialized || (entry == NULL)) {
        return APP_ERR_INVALID_PARAM;
    }

    if (!logger->file_open) {
        return APP_ERR_NOT_INIT;
    }

    /* Check distance threshold */
    if ((current_distance - logger->last_log_distance) < SD_LOG_MIN_DISTANCE_M) {
        return APP_OK; /* Not enough distance traveled */
    }

    /* Add to buffer */
    logger->buffer[logger->buffer_count] = *entry;
    logger->buffer_count++;
    logger->last_log_distance = current_distance;

    /* Check if buffer is full */
    if (logger->buffer_count >= SD_LOG_BUFFER_SIZE) {
        return write_buffer_to_file(logger);
    }

    return APP_OK;
}

app_err_t sd_logger_flush(sd_logger_t *logger)
{
    if ((logger == NULL) || !logger->is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (logger->buffer_count > 0U) {
        return write_buffer_to_file(logger);
    }

    return APP_OK;
}

uint32_t sd_logger_get_count(const sd_logger_t *logger)
{
    if (logger == NULL) {
        return 0U;
    }

    return logger->total_entries;
}

bool sd_logger_is_active(const sd_logger_t *logger)
{
    if (logger == NULL) {
        return false;
    }

    return logger->file_open;
}

const char *sd_logger_get_filename(const sd_logger_t *logger)
{
    if ((logger == NULL) || !logger->file_open) {
        return NULL;
    }

    return logger->filename;
}

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
                           float climb)
{
    if (entry == NULL) {
        return;
    }

    (void)memset(entry, 0, sizeof(sd_log_entry_t));

    if (loc != NULL) {
        entry->loc = *loc;
        entry->alti.gps_alt = loc->alt;
    }

    if (date != NULL) {
        entry->date = *date;
    }

    entry->sensors.power = power;
    entry->sensors.bpm = bpm;
    entry->sensors.cadence = cadence;
    entry->sensors.speed = speed;

    entry->alti.baro_alt = baro_alt;
    entry->alti.filt_alt = filt_alt;
    entry->alti.slope = slope;

    entry->distance = distance;
    entry->climb = climb;
    entry->timestamp = k_uptime_get_32();
}
