/**
 * @file locator.c
 * @brief Position locator implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

#include "model/locator.h"
#include "model/kalman.h"

LOG_MODULE_REGISTER(locator, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Minimum speed threshold for movement detection (km/h) */
#define MIN_SPEED_THRESHOLD     2.0f

/** Minimum distance for slope calculation (m) */
#define MIN_DIST_FOR_SLOPE      10.0f

/** History window for slope averaging */
#define SLOPE_AVG_WINDOW        5U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Position history buffer */
static history_point_t history[MAX_HISTORY_POINTS];
static uint8_t history_head;
static uint8_t history_count;

/** Total accumulated values */
static float total_distance;
static float total_climb;

/** Kalman filter for position smoothing */
static kalman_position_t pos_filter;

/** Kalman filter for altitude (separate for more smoothing) */
static kalman_state_t alt_filter;

/** Kalman filter for speed */
static kalman_state_t speed_filter;

/** Current computed values */
static float current_speed;
static int8_t current_slope;
static float current_vspeed;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Convert degrees to radians
 */
static inline float deg_to_rad(float deg)
{
    return deg * 0.017453292519943295f;
}

/**
 * @brief Add point to history buffer (circular)
 */
static void add_to_history(const history_point_t *point)
{
    history[history_head] = *point;
    history_head = (history_head + 1U) % MAX_HISTORY_POINTS;

    if (history_count < MAX_HISTORY_POINTS) {
        history_count++;
    }
}

/**
 * @brief Get history point by age (0 = most recent)
 */
static const history_point_t *get_history_by_age(uint8_t age)
{
    if (age >= history_count) {
        return NULL;
    }

    uint8_t idx = (history_head + MAX_HISTORY_POINTS - 1U - age) % MAX_HISTORY_POINTS;
    return &history[idx];
}

/**
 * @brief Calculate slope from recent history
 */
static void calculate_slope(void)
{
    if (history_count < 2U) {
        current_slope = 0;
        current_vspeed = 0.0f;
        return;
    }

    const history_point_t *newest = get_history_by_age(0);
    const history_point_t *oldest = NULL;
    float dist_sum = 0.0f;

    /* Find point far enough back for slope calculation */
    for (uint8_t i = 1U; (i < history_count) && (i < SLOPE_AVG_WINDOW); i++) {
        oldest = get_history_by_age(i);
        if (oldest != NULL) {
            dist_sum += oldest->dist_from_prev;
        }
        if (dist_sum >= MIN_DIST_FOR_SLOPE) {
            break;
        }
    }

    if ((oldest == NULL) || (newest == NULL) || (dist_sum < MIN_DIST_FOR_SLOPE)) {
        return;
    }

    /* Calculate slope */
    float elev_diff = newest->loc.alt - oldest->loc.alt;
    float slope_pct = (elev_diff / dist_sum) * 100.0f;

    /* Clamp slope to reasonable range */
    if (slope_pct > 45.0f) {
        slope_pct = 45.0f;
    } else if (slope_pct < -45.0f) {
        slope_pct = -45.0f;
    }

    current_slope = (int8_t)slope_pct;

    /* Calculate vertical speed */
    uint32_t time_diff = newest->loc.timestamp - oldest->loc.timestamp;
    if (time_diff > 0U) {
        current_vspeed = (elev_diff * 1000.0f) / (float)time_diff;  /* m/s */
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t locator_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Clear history */
    (void)memset(history, 0, sizeof(history));
    history_head = 0U;
    history_count = 0U;

    /* Reset totals */
    total_distance = 0.0f;
    total_climb = 0.0f;

    /* Initialize Kalman filters */
    kalman_position_init(&pos_filter);
    kalman_init(&alt_filter, 0.05f, 0.5f);  /* Extra smoothing for altitude */
    kalman_init(&speed_filter, 0.2f, 1.0f);

    /* Reset computed values */
    current_speed = 0.0f;
    current_slope = 0;
    current_vspeed = 0.0f;

    is_initialized = true;
    LOG_INF("Locator initialized");

    return APP_OK;
}

app_err_t locator_add_point(const loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (!locator_is_valid_position(loc)) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Apply Kalman filtering */
    loc_data_t filtered = kalman_position_update(&pos_filter, loc);

    /* Extra altitude smoothing */
    filtered.alt = kalman_update(&alt_filter, filtered.alt);

    /* Create history point */
    history_point_t point = {
        .loc = filtered,
        .dist_from_prev = 0.0f,
        .bearing = 0.0f,
        .valid = true,
    };

    /* Calculate distance and bearing from previous point */
    const history_point_t *prev = get_history_by_age(0);
    if ((prev != NULL) && prev->valid) {
        point.dist_from_prev = locator_calc_distance(
            prev->loc.lat, prev->loc.lon,
            filtered.lat, filtered.lon
        );

        point.bearing = locator_calc_bearing(
            prev->loc.lat, prev->loc.lon,
            filtered.lat, filtered.lon
        );

        /* Accumulate distance if moving */
        if (filtered.speed > MIN_SPEED_THRESHOLD) {
            total_distance += point.dist_from_prev;
        }

        /* Accumulate climb */
        float elev_diff = filtered.alt - prev->loc.alt;
        if (elev_diff > 0.5f) {  /* Minimum 0.5m to count */
            total_climb += elev_diff;
        }
    }

    /* Add to history */
    add_to_history(&point);

    /* Update current speed (smoothed) */
    current_speed = kalman_update(&speed_filter, filtered.speed);

    /* Calculate slope */
    calculate_slope();

    return APP_OK;
}

app_err_t locator_get_current(loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (loc == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    const history_point_t *current = get_history_by_age(0);
    if ((current == NULL) || !current->valid) {
        return APP_ERR_NOT_FOUND;
    }

    *loc = current->loc;
    return APP_OK;
}

app_err_t locator_get_history(uint8_t index, history_point_t *point)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (point == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    const history_point_t *hp = get_history_by_age(index);
    if ((hp == NULL) || !hp->valid) {
        return APP_ERR_NOT_FOUND;
    }

    *point = *hp;
    return APP_OK;
}

float locator_get_total_distance(void)
{
    return total_distance;
}

float locator_get_total_climb(void)
{
    return total_climb;
}

float locator_get_speed(void)
{
    return current_speed;
}

int8_t locator_get_slope(void)
{
    return current_slope;
}

float locator_get_vspeed(void)
{
    return current_vspeed;
}

float locator_calc_distance(float lat1, float lon1, float lat2, float lon2)
{
    /* Haversine formula */
    float dlat = deg_to_rad(lat2 - lat1);
    float dlon = deg_to_rad(lon2 - lon1);

    float a = sinf(dlat / 2.0f) * sinf(dlat / 2.0f) +
              cosf(deg_to_rad(lat1)) * cosf(deg_to_rad(lat2)) *
              sinf(dlon / 2.0f) * sinf(dlon / 2.0f);

    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    return EARTH_RADIUS_M * c;
}

float locator_calc_bearing(float lat1, float lon1, float lat2, float lon2)
{
    float dlon = deg_to_rad(lon2 - lon1);
    float lat1_rad = deg_to_rad(lat1);
    float lat2_rad = deg_to_rad(lat2);

    float x = sinf(dlon) * cosf(lat2_rad);
    float y = cosf(lat1_rad) * sinf(lat2_rad) -
              sinf(lat1_rad) * cosf(lat2_rad) * cosf(dlon);

    float bearing = atan2f(x, y);

    /* Normalize to 0-2*PI */
    if (bearing < 0.0f) {
        bearing += 2.0f * 3.14159265358979323846f;
    }

    return bearing;
}

bool locator_is_valid_position(const loc_data_t *loc)
{
    if (loc == NULL) {
        return false;
    }

    /* Check latitude range */
    if ((loc->lat < -90.0f) || (loc->lat > 90.0f)) {
        return false;
    }

    /* Check longitude range */
    if ((loc->lon < -180.0f) || (loc->lon > 180.0f)) {
        return false;
    }

    /* Check for null island (common GPS error) */
    if ((fabsf(loc->lat) < 0.0001f) && (fabsf(loc->lon) < 0.0001f)) {
        return false;
    }

    return true;
}

void locator_reset(void)
{
    (void)memset(history, 0, sizeof(history));
    history_head = 0U;
    history_count = 0U;

    total_distance = 0.0f;
    total_climb = 0.0f;
    current_speed = 0.0f;
    current_slope = 0;
    current_vspeed = 0.0f;

    kalman_position_reset(&pos_filter);
    kalman_reset(&alt_filter);
    kalman_reset(&speed_filter);

    LOG_INF("Locator reset");
}

uint8_t locator_get_history_count(void)
{
    return history_count;
}
