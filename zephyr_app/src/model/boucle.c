/**
 * @file boucle.c
 * @brief Main application loop controller implementation
 *
 * Integrates all sensor data and manages activity recording.
 * Based on original BoucleCRS.cpp implementation.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <string.h>
#include <stdio.h>

#include "model/boucle.h"
#include "model/attitude.h"
#include "model/segment.h"
#include "model/locator.h"
#include "model/sd_logger.h"
#include "model/power_zone.h"
#include "model/suffer_score.h"
#include "model/parcours.h"
#include "drivers/gps_mgmt.h"
#include "drivers/baro.h"
#include "drivers/fxos.h"
#include "drivers/stc3100.h"

LOG_MODULE_REGISTER(boucle, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Sensor polling interval in milliseconds */
#define SENSOR_POLL_INTERVAL_MS     100U

/** GPS polling interval in milliseconds */
#define GPS_POLL_INTERVAL_MS        1000U

/** Display update interval in milliseconds */
#define DISPLAY_UPDATE_INTERVAL_MS  250U

/** Segment allocation distance (from legacy) */
#define DIST_ALLOC                  3000.0f

/** Segment margin for deactivation */
#define MARGE_DESACT                1.5f

/** Default FTP for power zones */
#define DEFAULT_FTP_WATTS           200U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current state */
static boucle_state_t current_state = BOUCLE_STATE_IDLE;

/** Current application mode */
static app_mode_t current_mode = APP_MODE_INIT;

/** Statistics */
static boucle_stats_t stats;

/** Last GPS update time */
static uint32_t last_gps_time;

/** Last sensor update time */
static uint32_t last_sensor_time;

/** Initialization flag */
static bool is_initialized;

/** SD Logger instance */
static sd_logger_t sd_logger;

/** Power zone tracker */
static power_zone_t power_zones;

/** Suffer score tracker */
static suffer_score_t suffer_score;

/** Distance to nearest segment */
static float dist_next_segment;

/** Current HRM data */
static hrm_info_t hrm_data;

/** Current BSC data */
static bsc_info_t bsc_data;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Log current position and sensor data to SD
 */
static void log_position_data(const loc_data_t *loc)
{
    if (!sd_logger_is_active(&sd_logger)) {
        return;
    }

    attitude_t att;
    if (attitude_get(&att) != APP_OK) {
        return;
    }

    /* Build log entry from current state */
    sd_log_entry_t entry;
    sd_logger_build_entry(&entry,
                          loc,
                          &att.date,
                          att.pwr,
                          hrm_data.bpm,
                          bsc_data.cadence,
                          (uint16_t)(loc->speed * 100.0f),
                          loc->alt,           /* baro_alt approximation */
                          loc->alt,           /* filt_alt */
                          att.slope,
                          att.dist,
                          att.climb);

    /* Add to logger (respects distance threshold internally) */
    (void)sd_logger_add_entry(&sd_logger, &entry, att.dist);
}

/**
 * @brief Update power and HR zone trackers
 */
static void update_zone_trackers(uint16_t power, uint8_t bpm)
{
    uint32_t now = k_uptime_get_32();

    /* Update power zones if power is available */
    if (power > 0U) {
        power_zone_add_data(&power_zones, power, now);
    }

    /* Update suffer score if HR is available */
    if (bpm > 0U) {
        suffer_score_add_hrm(&suffer_score, bpm, now);
    }
}

/**
 * @brief GPS fix callback
 */
static void gps_fix_callback(const gps_data_t *data)
{
    if (data == NULL) {
        return;
    }

    if (current_state == BOUCLE_STATE_RUNNING) {
        /* Update attitude with GPS data */
        (void)attitude_update_gps(&data->location);

        /* Update segments */
        (void)segment_update(&data->location);

        /* Update stats */
        stats.gps_points++;
        stats.total_distance = attitude_get_distance();
        stats.total_climb = attitude_get_climb();
        stats.active_segments = segment_get_active_count();

        /* Update max speed */
        if (data->location.speed > stats.max_speed) {
            stats.max_speed = data->location.speed;
        }

        /* Update distance to nearest segment */
        dist_next_segment = segment_get_nearest_distance();

        /* Log position data to SD card */
        log_position_data(&data->location);

        /* Update zone trackers */
        update_zone_trackers(attitude_get_power(), hrm_data.bpm);

        /* Update parcours if active */
        if ((current_mode == APP_MODE_PRC) && parcours_is_active()) {
            parcours_update(data->location.lat, data->location.lon,
                           data->location.alt);
        }

        LOG_DBG("GPS: %.5f,%.5f spd=%.1f dist=%.0f climb=%.0f",
                (double)data->location.lat, (double)data->location.lon,
                (double)data->location.speed, (double)stats.total_distance,
                (double)stats.total_climb);
    }
}

/**
 * @brief Poll sensors
 */
static void poll_sensors(void)
{
    /* Barometer */
    if (baro_trigger() == APP_OK) {
        baro_data_t baro_data;
        if (baro_read(&baro_data) == APP_OK) {
            (void)attitude_update_baro(baro_data.pressure, baro_data.temperature);
        }
    }

    /* IMU */
    if (fxos_trigger() == APP_OK) {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;

        (void)fxos_get_yaw(&yaw);
        (void)fxos_get_pitch(&pitch);
        (void)fxos_get_roll(&roll);

        /* Convert radians to degrees */
        (void)attitude_update_imu(yaw * 57.2957795f, pitch * 57.2957795f,
                                  roll * 57.2957795f);
    }

    /* Battery */
    if (stc3100_trigger() == APP_OK) {
        attitude_update_battery(stc3100_get_soc(), stc3100_get_voltage());
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t boucle_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Initialize sub-modules */
    app_err_t err = attitude_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_ERR("Failed to init attitude: %d", err);
        return err;
    }

    err = segment_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_ERR("Failed to init segment: %d", err);
        return err;
    }

    /* Initialize parcours module */
    err = parcours_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("Failed to init parcours: %d", err);
    }

    /* Initialize SD logger */
    err = sd_logger_init(&sd_logger);
    if (err != APP_OK) {
        LOG_WRN("Failed to init SD logger: %d", err);
    }

    /* Initialize power zones with default FTP */
    power_zone_init(&power_zones, DEFAULT_FTP_WATTS);

    /* Initialize suffer score tracker */
    suffer_score_init(&suffer_score);

    /* Register GPS callback */
    err = gps_mgmt_register_callback(gps_fix_callback);
    if (err != APP_OK) {
        LOG_WRN("Failed to register GPS callback: %d", err);
    }

    /* Clear stats and sensor data */
    (void)memset(&stats, 0, sizeof(stats));
    (void)memset(&hrm_data, 0, sizeof(hrm_data));
    (void)memset(&bsc_data, 0, sizeof(bsc_data));
    dist_next_segment = 9999.0f;

    current_state = BOUCLE_STATE_IDLE;
    current_mode = APP_MODE_INIT;
    is_initialized = true;

    LOG_INF("Boucle initialized (FTP=%u)", DEFAULT_FTP_WATTS);

    return APP_OK;
}

app_err_t boucle_start(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (current_state == BOUCLE_STATE_RUNNING) {
        return APP_OK;  /* Already running */
    }

    /* Reset if was idle */
    if (current_state == BOUCLE_STATE_IDLE) {
        boucle_reset_stats();

        /* Reset zone trackers */
        power_zone_reset(&power_zones);
        suffer_score_reset(&suffer_score);

        /* Start SD logging with current date */
        attitude_t att;
        if (attitude_get(&att) == APP_OK) {
            (void)sd_logger_start(&sd_logger, &att.date);
        } else {
            (void)sd_logger_start(&sd_logger, NULL);
        }
    }

    /* Start GPS */
    (void)gps_mgmt_start();

    /* Record start time */
    stats.start_time = k_uptime_get_32();

    current_state = BOUCLE_STATE_RUNNING;
    LOG_INF("Activity started");

    return APP_OK;
}

app_err_t boucle_stop(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (current_state == BOUCLE_STATE_IDLE) {
        return APP_OK;
    }

    /* Stop GPS */
    (void)gps_mgmt_stop();

    /* Stop SD logging - flushes remaining buffer */
    (void)sd_logger_stop(&sd_logger);

    /* Calculate final stats */
    stats.elapsed_time = attitude_get_elapsed_time();
    stats.moving_time = attitude_get_moving_time();
    stats.avg_speed = attitude_get_avg_speed();

    /* Log final statistics */
    LOG_INF("Activity stopped - dist=%.1f km, climb=%.0f m, time=%u s",
            (double)(stats.total_distance / 1000.0f),
            (double)stats.total_climb,
            stats.elapsed_time);
    LOG_INF("Suffer score: %.1f, Power zone total: %u s",
            (double)suffer_score_get(&suffer_score),
            power_zone_get_total_time(&power_zones));

    current_state = BOUCLE_STATE_IDLE;

    return APP_OK;
}

app_err_t boucle_pause(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (current_state != BOUCLE_STATE_RUNNING) {
        return APP_ERR_BUSY;
    }

    /* Put GPS in standby */
    (void)gps_mgmt_standby();

    current_state = BOUCLE_STATE_PAUSED;
    LOG_INF("Activity paused");

    return APP_OK;
}

app_err_t boucle_resume(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (current_state != BOUCLE_STATE_PAUSED) {
        return APP_ERR_BUSY;
    }

    /* Wake GPS */
    (void)gps_mgmt_wake();

    current_state = BOUCLE_STATE_RUNNING;
    LOG_INF("Activity resumed");

    return APP_OK;
}

boucle_state_t boucle_get_state(void)
{
    return current_state;
}

app_err_t boucle_get_stats(boucle_stats_t *out_stats)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (out_stats == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Update live stats */
    stats.elapsed_time = attitude_get_elapsed_time();
    stats.moving_time = attitude_get_moving_time();
    stats.avg_speed = attitude_get_avg_speed();

    *out_stats = stats;
    return APP_OK;
}

void boucle_process(void)
{
    if (!is_initialized) {
        return;
    }

    uint32_t now = k_uptime_get_32();

    /* Poll sensors at regular interval */
    if ((now - last_sensor_time) >= SENSOR_POLL_INTERVAL_MS) {
        poll_sensors();
        last_sensor_time = now;
    }

    /* Process GPS */
    if ((now - last_gps_time) >= GPS_POLL_INTERVAL_MS) {
        gps_mgmt_process();
        last_gps_time = now;
    }

    /* Compute attitude derived values */
    attitude_compute();
}

void boucle_handle_button(btn_event_t event)
{
    if (!is_initialized) {
        return;
    }

    switch (event) {
    case BTN_EVENT_CENTER:
        /* Start/Stop toggle */
        if (current_state == BOUCLE_STATE_IDLE) {
            (void)boucle_start();
        } else if (current_state == BOUCLE_STATE_RUNNING) {
            (void)boucle_pause();
        } else if (current_state == BOUCLE_STATE_PAUSED) {
            (void)boucle_resume();
        }
        break;

    case BTN_EVENT_LONG_CENTER:
        /* Stop and save */
        if (current_state != BOUCLE_STATE_IDLE) {
            (void)boucle_stop();
            (void)boucle_save_activity();
        }
        break;

    case BTN_EVENT_LEFT:
    case BTN_EVENT_RIGHT:
        /* Mode/page navigation - handled by vue layer */
        break;

    default:
        break;
    }
}

app_err_t boucle_set_mode(app_mode_t mode)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    current_mode = mode;
    LOG_INF("Mode changed to %d", mode);

    return APP_OK;
}

app_mode_t boucle_get_mode(void)
{
    return current_mode;
}

void boucle_reset_stats(void)
{
    (void)memset(&stats, 0, sizeof(stats));
    attitude_reset();
    segment_reset_all();

    LOG_INF("Stats reset");
}

app_err_t boucle_save_activity(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    current_state = BOUCLE_STATE_SAVING;
    LOG_INF("Saving activity summary...");

    /* Get final stats */
    attitude_t att;
    if (attitude_get(&att) != APP_OK) {
        current_state = BOUCLE_STATE_IDLE;
        return APP_ERR_IO;
    }

    /* Write activity summary file */
    struct fs_file_t file;
    char filename[64];
    char buffer[256];
    int ret;

    /* Generate filename from date */
    uint32_t day = att.date.date / 10000U;
    uint32_t month = (att.date.date / 100U) % 100U;
    uint32_t year = att.date.date % 100U;

    (void)snprintf(filename, sizeof(filename),
                   "/SD:/logs/ACT_%02u%02u%02u.txt",
                   (unsigned)year, (unsigned)month, (unsigned)day);

    fs_file_t_init(&file);
    ret = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret < 0) {
        LOG_ERR("Failed to create activity file: %d", ret);
        current_state = BOUCLE_STATE_IDLE;
        return APP_ERR_IO;
    }

    /* Write header */
    int len = snprintf(buffer, sizeof(buffer),
                       "=== Activity Summary ===\r\n"
                       "Date: %u\r\n"
                       "Time: %u s after midnight\r\n\r\n",
                       (unsigned)att.date.date, (unsigned)att.date.secj);
    (void)fs_write(&file, buffer, (size_t)len);

    /* Write distance and time stats */
    len = snprintf(buffer, sizeof(buffer),
                   "[Distance & Time]\r\n"
                   "Total Distance: %.2f km\r\n"
                   "Total Climb: %.0f m\r\n"
                   "Elapsed Time: %u s\r\n"
                   "Moving Time: %u s\r\n"
                   "GPS Points: %u\r\n\r\n",
                   (double)(stats.total_distance / 1000.0f),
                   (double)stats.total_climb,
                   stats.elapsed_time,
                   stats.moving_time,
                   (unsigned)stats.gps_points);
    (void)fs_write(&file, buffer, (size_t)len);

    /* Write speed stats */
    len = snprintf(buffer, sizeof(buffer),
                   "[Speed]\r\n"
                   "Max Speed: %.1f km/h\r\n"
                   "Avg Speed: %.1f km/h\r\n\r\n",
                   (double)stats.max_speed,
                   (double)stats.avg_speed);
    (void)fs_write(&file, buffer, (size_t)len);

    /* Write power zones */
    len = snprintf(buffer, sizeof(buffer),
                   "[Power Zones (s)]\r\n"
                   "Z1 (Recovery): %u\r\n"
                   "Z2 (Endurance): %u\r\n"
                   "Z3 (Tempo): %u\r\n"
                   "Z4 (Threshold): %u\r\n"
                   "Z5 (VO2max): %u\r\n"
                   "Z6 (Anaerobic): %u\r\n"
                   "Z7 (Neuromuscular): %u\r\n"
                   "Total: %u\r\n\r\n",
                   power_zone_get_time(&power_zones, 0U),
                   power_zone_get_time(&power_zones, 1U),
                   power_zone_get_time(&power_zones, 2U),
                   power_zone_get_time(&power_zones, 3U),
                   power_zone_get_time(&power_zones, 4U),
                   power_zone_get_time(&power_zones, 5U),
                   power_zone_get_time(&power_zones, 6U),
                   power_zone_get_total_time(&power_zones));
    (void)fs_write(&file, buffer, (size_t)len);

    /* Write HR zones and suffer score */
    len = snprintf(buffer, sizeof(buffer),
                   "[HR Zones (s)]\r\n"
                   "Z1 (80-120): %u\r\n"
                   "Z2 (120-144): %u\r\n"
                   "Z3 (144-165): %u\r\n"
                   "Z4 (165-176): %u\r\n"
                   "Z5 (>176): %u\r\n"
                   "Total: %u\r\n"
                   "Suffer Score: %.1f\r\n\r\n",
                   suffer_score_get_zone_time(&suffer_score, 0U),
                   suffer_score_get_zone_time(&suffer_score, 1U),
                   suffer_score_get_zone_time(&suffer_score, 2U),
                   suffer_score_get_zone_time(&suffer_score, 3U),
                   suffer_score_get_zone_time(&suffer_score, 4U),
                   suffer_score_get_total_time(&suffer_score),
                   (double)suffer_score_get(&suffer_score));
    (void)fs_write(&file, buffer, (size_t)len);

    /* Write segment info */
    len = snprintf(buffer, sizeof(buffer),
                   "[Segments]\r\n"
                   "Active Segments: %u\r\n"
                   "PRs: %u\r\n",
                   (unsigned)stats.active_segments,
                   (unsigned)stats.pr_count);
    (void)fs_write(&file, buffer, (size_t)len);

    (void)fs_close(&file);

    LOG_INF("Activity saved to %s", filename);
    current_state = BOUCLE_STATE_IDLE;

    return APP_OK;
}

bool boucle_is_active(void)
{
    return (current_state == BOUCLE_STATE_RUNNING) ||
           (current_state == BOUCLE_STATE_PAUSED);
}

app_err_t boucle_get_attitude(attitude_t *att)
{
    return attitude_get(att);
}

void boucle_update_hrm(uint8_t bpm, bool connected)
{
    hrm_data.bpm = bpm;
    hrm_data.connected = connected;

    /* Update attitude as well */
    attitude_update_hrm(bpm, connected);
}

void boucle_update_bsc(uint8_t cadence, uint16_t speed, bool connected)
{
    bsc_data.cadence = cadence;
    bsc_data.speed = speed;
    bsc_data.connected = connected;
}

float boucle_get_suffer_score(void)
{
    return suffer_score_get(&suffer_score);
}

uint8_t boucle_get_power_zone(void)
{
    return power_zone_get_current(&power_zones);
}

float boucle_get_dist_to_segment(void)
{
    return dist_next_segment;
}

void boucle_set_ftp(uint16_t ftp)
{
    power_zone_set_ftp(&power_zones, ftp);
}
