/**
 * @file attitude.c
 * @brief Attitude manager implementation
 *
 * Based on original Attitude.cpp with 3-state Kalman filter for
 * altitude fusion using barometer and accelerometer pitch.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include "model/attitude.h"
#include "model/locator.h"
#include "model/kalman_altitude.h"
#include "model/crash_recovery.h"

LOG_MODULE_REGISTER(attitude, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Minimum speed for moving time (km/h) */
#define MOVING_SPEED_THRESHOLD  2.0f

/** Minimum speed for Kalman update (m/s) */
#define MIN_SPEED_FOR_KALMAN    1.5f

/** Power estimation constants */
#define RIDER_WEIGHT_KG         75.0f
#define BIKE_WEIGHT_KG          10.0f
#define ROLLING_RESISTANCE      0.005f
#define AIR_DENSITY             1.225f
#define DRAG_COEFF              0.3f
#define FRONTAL_AREA            0.5f

/** Climb calculation hysteresis (meters) - from original */
#define CLIMB_ELEVATION_HYSTERESIS_M    2.0f

/** GPS points before slope calculation is valid */
#define MIN_PTS_FOR_SLOPE       60U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current attitude data */
static attitude_t current_att;

/** Extended attitude data */
static attitude_ext_t current_ext;

/** Start timestamp */
static uint32_t start_time;

/** Last update timestamp */
static uint32_t last_update;

/** Elapsed time counters */
static uint32_t elapsed_seconds;
static uint32_t moving_seconds;

/** Speed accumulator for average */
static float speed_sum;
static uint32_t speed_count;

/** Kalman filter for altitude fusion */
static kalman_altitude_t altitude_kf;

/** Current filtered elevation */
static float current_elevation;

/** Last stored elevation for climb calculation */
static float last_stored_elevation;

/** Accumulated climb */
static float accumulated_climb;

/** Current speed in m/s */
static float current_speed_ms;

/** Last Kalman update time */
static uint32_t last_kalman_time;

/** Current pitch from IMU (radians) */
static float current_pitch_rad;

/** Sea level pressure reference */
static float sea_level_pressure;

/** Baro/GPS altitude correction filter */
static float altitude_correction;

/** Initialization flags */
static bool is_initialized;
static bool is_altitude_initialized;
static bool has_sea_level_ref;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Compute barometric altitude from pressure
 * @param pressure Pressure in Pa
 * @return Altitude in meters
 */
static float compute_baro_altitude(float pressure)
{
    if (pressure <= 0.0f) {
        return 0.0f;
    }

    /* Using standard atmosphere if no sea level ref */
    float p0 = (sea_level_pressure > 0.0f) ? sea_level_pressure : 101325.0f;

    /* h = 44330 * (1 - (P/P0)^0.1903) */
    float altitude = 44330.0f * (1.0f - powf(pressure / p0, 0.1903f));

    /* Apply GPS/baro correction */
    return altitude - altitude_correction;
}

/**
 * @brief Filter altitude using Kalman and update climb
 *
 * Implements the 3-state Kalman filter for altitude fusion:
 * State: [elevation, pitch, alpha_zero]
 * Model: d(elevation) = (pitch - alpha_zero) * dl
 */
static void compute_altitude_fusion(void)
{
    /* Get current barometric altitude */
    float baro_ele = compute_baro_altitude(current_ext.pressure);

    /* Initialize Kalman if not done */
    if (!altitude_kf.is_initialized) {
        kalman_altitude_init(&altitude_kf, baro_ele);
        last_kalman_time = k_uptime_get_32();
        LOG_INF("Kalman altitude filter initialized (ele=%.1f)", (double)baro_ele);
        return;
    }

    /* Skip if not moving fast enough */
    if (current_speed_ms < MIN_SPEED_FOR_KALMAN) {
        last_kalman_time = k_uptime_get_32();
        return;
    }

    /* Calculate time delta and distance traveled */
    uint32_t now = k_uptime_get_32();
    float dt_sec = (float)(now - last_kalman_time) / 1000.0f;
    float dl = current_speed_ms * dt_sec;

    if (dl < 0.1f) {
        return; /* Not enough movement */
    }

    /* Feed Kalman filter */
    kalman_altitude_feed(&altitude_kf, baro_ele, current_pitch_rad, dl);

    /* Get filtered values */
    current_elevation = kalman_altitude_get_elevation(&altitude_kf);
    float alpha_bar = kalman_altitude_get_pitch(&altitude_kf);
    float alpha_zero = kalman_altitude_get_alpha_zero(&altitude_kf);

    last_kalman_time = now;

    /* Update slope and vertical speed after enough data points */
    if (current_att.nbpts > MIN_PTS_FOR_SLOPE) {
        float slope = alpha_bar - alpha_zero;
        current_att.slope = (int8_t)(100.0f * slope);
        current_att.vit_asc = slope * current_speed_ms;
    }

    LOG_DBG("Fusion: ele=%.1f slope=%d vit_asc=%.2f alpha0=%.3f",
            (double)current_elevation, current_att.slope,
            (double)current_att.vit_asc, (double)alpha_zero);
}

/**
 * @brief Update climb calculation with hysteresis
 */
static void update_climb(void)
{
    if (!is_altitude_initialized) {
        last_stored_elevation = current_elevation;
        accumulated_climb = 0.0f;
        is_altitude_initialized = true;
        return;
    }

    /* Only count climb with hysteresis to avoid noise */
    if (current_elevation > last_stored_elevation + CLIMB_ELEVATION_HYSTERESIS_M) {
        /* Going up - add to accumulated climb */
        accumulated_climb += current_elevation - last_stored_elevation;
        last_stored_elevation = current_elevation;
    } else if (current_elevation + CLIMB_ELEVATION_HYSTERESIS_M < last_stored_elevation) {
        /* Going down - update reference without adding climb */
        last_stored_elevation = current_elevation;
    }

    /* Sanity check */
    if (accumulated_climb < 0.0f) {
        accumulated_climb = 0.0f;
        is_altitude_initialized = false;
    }

    current_att.climb = accumulated_climb;
}

/**
 * @brief Filter GPS/baro altitude difference to remove drift
 * @param gps_alt GPS altitude
 * @param baro_alt Barometer altitude
 */
static void filter_altitude_correction(float gps_alt, float baro_alt)
{
    /* High time-constant filter (about 800 samples) */
    const float tau = 800.0f / (800.0f + 1.0f);
    float input = baro_alt - gps_alt;

    altitude_correction = tau * altitude_correction + (1.0f - tau) * input;

    LOG_DBG("Alt correction: gps=%.1f baro=%.1f corr=%.2f",
            (double)gps_alt, (double)baro_alt, (double)altitude_correction);
}

/**
 * @brief Save current state for crash recovery
 */
static void save_crash_recovery_state(void)
{
    crash_recovery_save_state(&current_att.loc,
                              &current_att.date,
                              current_att.dist,
                              current_att.climb,
                              current_att.nbpts,
                              current_att.nbsec_act);
}

/**
 * @brief Estimate cycling power
 */
static uint16_t estimate_power(float speed_kmh, int8_t slope)
{
    if (speed_kmh < 0.5f) {
        return 0U;
    }

    float speed_ms = speed_kmh / 3.6f;
    float total_mass = RIDER_WEIGHT_KG + BIKE_WEIGHT_KG;

    /* Rolling resistance power */
    float p_roll = ROLLING_RESISTANCE * total_mass * 9.81f * speed_ms;

    /* Air resistance power */
    float p_air = 0.5f * AIR_DENSITY * DRAG_COEFF * FRONTAL_AREA *
                  speed_ms * speed_ms * speed_ms;

    /* Gravity power (climbing/descending) */
    float grade = (float)slope / 100.0f;
    float p_gravity = total_mass * 9.81f * grade * speed_ms;

    /* Total power (minimum 0) */
    float power = p_roll + p_air + p_gravity;
    if (power < 0.0f) {
        power = 0.0f;
    }

    return (uint16_t)power;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t attitude_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Initialize locator */
    app_err_t err = locator_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        return err;
    }

    /* Initialize crash recovery */
    (void)crash_recovery_init();

    /* Clear data */
    (void)memset(&current_att, 0, sizeof(current_att));
    (void)memset(&current_ext, 0, sizeof(current_ext));
    (void)memset(&altitude_kf, 0, sizeof(altitude_kf));

    start_time = 0U;
    last_update = 0U;
    elapsed_seconds = 0U;
    moving_seconds = 0U;
    speed_sum = 0.0f;
    speed_count = 0U;

    current_elevation = 0.0f;
    last_stored_elevation = 0.0f;
    accumulated_climb = 0.0f;
    current_speed_ms = 0.0f;
    last_kalman_time = 0U;
    current_pitch_rad = 0.0f;
    sea_level_pressure = 0.0f;
    altitude_correction = 0.0f;

    is_altitude_initialized = false;
    has_sea_level_ref = false;

    /* Check for crash recovery data */
    if (crash_recovery_has_data()) {
        saved_data_t saved;
        if (crash_recovery_get_saved_state(&saved)) {
            LOG_WRN("Restoring data from crash recovery");
            LOG_WRN("Distance: %.1f m, Climb: %.1f m",
                    (double)saved.dist, (double)saved.climb);

            current_att.dist = saved.dist;
            current_att.climb = accumulated_climb = saved.climb;
            current_att.nbpts = saved.nbpts;
            current_att.nbsec_act = saved.nbsec_act;

            crash_recovery_clear();
        }
    }

    is_initialized = true;
    LOG_INF("Attitude manager initialized");

    return APP_OK;
}

app_err_t attitude_update_gps(const loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (loc == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Update speed (m/s) */
    current_speed_ms = loc->speed / 3.6f;

    /* Add to locator for distance calculation */
    app_err_t err = locator_add_point(loc);
    if (err != APP_OK) {
        return err;
    }

    /* Update current attitude */
    current_att.loc = *loc;
    current_att.dist = locator_get_total_distance();
    current_att.nbpts++;

    /* Initialize sea level pressure if barometer ready and enough GPS points */
    if (!has_sea_level_ref &&
        (current_ext.pressure > 0.0f) &&
        (current_att.nbpts > 15U)) {

        /* Calculate sea level pressure from GPS altitude */
        /* P0 = P / (1 - alt/44330)^5.255 */
        float alt = loc->alt;
        float factor = 1.0f - (alt / 44330.0f);
        if (factor > 0.0f) {
            sea_level_pressure = current_ext.pressure / powf(factor, 5.255f);
            has_sea_level_ref = true;

            /* Initialize altitude */
            current_elevation = alt;
            is_altitude_initialized = false;

            LOG_INF("Sea level pressure initialized: %.1f Pa (GPS alt: %.1f m)",
                    (double)sea_level_pressure, (double)alt);
        }
    }

    /* Apply GPS/baro altitude correction filter */
    if (has_sea_level_ref && (current_ext.pressure > 0.0f)) {
        float baro_alt = compute_baro_altitude(current_ext.pressure);
        filter_altitude_correction(loc->alt, baro_alt);
    }

    /* Update climb from Kalman-filtered altitude */
    update_climb();
    current_att.climb = accumulated_climb;

    /* Update extended data */
    current_ext.base = current_att;

    /* Track moving time */
    uint32_t now = k_uptime_get_32();
    if (start_time == 0U) {
        start_time = now;
    }

    if ((now - last_update) >= 1000U) {  /* 1 second elapsed */
        elapsed_seconds++;

        if (loc->speed > MOVING_SPEED_THRESHOLD) {
            moving_seconds++;
            current_att.nbsec_act = (uint16_t)moving_seconds;
        }

        last_update = now;

        /* Save state for crash recovery periodically */
        save_crash_recovery_state();
    }

    /* Accumulate speed for average */
    if (loc->speed > 0.0f) {
        speed_sum += loc->speed;
        speed_count++;
    }

    /* Estimate power */
    current_att.pwr = estimate_power(loc->speed, current_att.slope);

    return APP_OK;
}

app_err_t attitude_update_baro(float pressure, float temperature)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    current_ext.pressure = pressure;
    current_ext.temperature = temperature;

    /* Calculate barometric altitude using calibrated sea level if available */
    if (pressure > 0.0f) {
        current_ext.baro_altitude = compute_baro_altitude(pressure);
    }

    /* Run altitude fusion if we have sea level reference */
    if (has_sea_level_ref) {
        compute_altitude_fusion();
    }

    return APP_OK;
}

app_err_t attitude_update_imu(float heading, float pitch, float roll)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    current_ext.heading = heading;

    /* Store pitch in radians for Kalman filter */
    /* The pitch angle is used to estimate slope when moving */
    current_pitch_rad = pitch * (3.14159265f / 180.0f);

    (void)roll; /* Roll not used currently */

    return APP_OK;
}

void attitude_update_battery(uint8_t soc, float voltage)
{
    if (!is_initialized) {
        return;
    }

    current_ext.battery_soc = soc;
    current_ext.battery_voltage = voltage;
}

void attitude_update_hrm(uint8_t bpm, bool connected)
{
    if (!is_initialized) {
        return;
    }

    current_ext.hrm_bpm = bpm;
    current_ext.hrm_connected = connected;
}

void attitude_update_datetime(const date_data_t *date)
{
    if (!is_initialized || (date == NULL)) {
        return;
    }

    current_att.date = *date;
}

app_err_t attitude_get(attitude_t *att)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (att == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *att = current_att;
    return APP_OK;
}

app_err_t attitude_get_ext(attitude_ext_t *att)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (att == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *att = current_ext;
    return APP_OK;
}

app_err_t attitude_get_location(loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (loc == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *loc = current_att.loc;
    return APP_OK;
}

float attitude_get_distance(void)
{
    return current_att.dist;
}

float attitude_get_climb(void)
{
    return current_att.climb;
}

uint32_t attitude_get_elapsed_time(void)
{
    return elapsed_seconds;
}

uint32_t attitude_get_moving_time(void)
{
    return moving_seconds;
}

float attitude_get_avg_speed(void)
{
    if (speed_count == 0U) {
        return 0.0f;
    }

    return speed_sum / (float)speed_count;
}

uint16_t attitude_get_power(void)
{
    return current_att.pwr;
}

void attitude_reset(void)
{
    (void)memset(&current_att, 0, sizeof(current_att));
    (void)memset(&current_ext, 0, sizeof(current_ext));
    (void)memset(&altitude_kf, 0, sizeof(altitude_kf));

    start_time = 0U;
    last_update = 0U;
    elapsed_seconds = 0U;
    moving_seconds = 0U;
    speed_sum = 0.0f;
    speed_count = 0U;

    current_elevation = 0.0f;
    last_stored_elevation = 0.0f;
    accumulated_climb = 0.0f;
    current_speed_ms = 0.0f;
    last_kalman_time = 0U;
    altitude_correction = 0.0f;

    is_altitude_initialized = false;
    has_sea_level_ref = false;

    locator_reset();
    crash_recovery_clear();

    LOG_INF("Attitude reset");
}

void attitude_compute(void)
{
    if (!is_initialized) {
        return;
    }

    /* Update elapsed time */
    if (start_time > 0U) {
        uint32_t now = k_uptime_get_32();
        elapsed_seconds = (now - start_time) / 1000U;
    }

    /* Copy base to extended */
    current_ext.base = current_att;
}
