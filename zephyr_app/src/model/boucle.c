/**
 * @file boucle.c
 * @brief Main application loop controller implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "model/boucle.h"
#include "model/attitude.h"
#include "model/segment.h"
#include "model/locator.h"
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

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

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

    /* Register GPS callback */
    err = gps_mgmt_register_callback(gps_fix_callback);
    if (err != APP_OK) {
        LOG_WRN("Failed to register GPS callback: %d", err);
    }

    /* Clear stats */
    (void)memset(&stats, 0, sizeof(stats));

    current_state = BOUCLE_STATE_IDLE;
    current_mode = APP_MODE_INIT;
    is_initialized = true;

    LOG_INF("Boucle initialized");

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

    /* Calculate final stats */
    stats.elapsed_time = attitude_get_elapsed_time();
    stats.moving_time = attitude_get_moving_time();
    stats.avg_speed = attitude_get_avg_speed();

    current_state = BOUCLE_STATE_IDLE;
    LOG_INF("Activity stopped");

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

    /* TODO: Implement activity saving to SD card */
    /* This would write a FIT or GPX file */

    current_state = BOUCLE_STATE_SAVING;

    LOG_INF("Saving activity...");

    /* Placeholder - actual implementation would write to file */
    k_msleep(100);

    current_state = BOUCLE_STATE_IDLE;
    LOG_INF("Activity saved");

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
