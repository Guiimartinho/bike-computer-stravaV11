/**
 * @file zwift.c
 * @brief Zwift indoor simulation mode implementation
 *
 * Handles indoor trainer simulation receiving position and metrics
 * from Zwift via BLE Nordic UART Service (NUS).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>

#include "model/zwift.h"
#include "model/attitude.h"
#include "model/boucle.h"
#include "drivers/gps_mgmt.h"
#include "rf/ble_fec_client.h"

LOG_MODULE_REGISTER(zwift, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Zwift packet header identifier */
#define ZWIFT_PACKET_HEADER     0x5A    /* 'Z' */

/** Zwift packet types */
#define ZWIFT_PKT_POSITION      0x01
#define ZWIFT_PKT_POWER         0x02
#define ZWIFT_PKT_GRADIENT      0x03

/** Data timeout in milliseconds */
#define ZWIFT_DATA_TIMEOUT_MS   5000U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current Zwift data */
static zwift_data_t zwift_data;

/** Active flag */
static bool is_active;

/** Initialization flag */
static bool is_initialized;

/** Target gradient for FEC */
static float target_gradient;

/** Target power for FEC */
static uint16_t target_power;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Parse position packet from Zwift
 */
static void parse_position_packet(const uint8_t *data, uint16_t len)
{
    /* Expected format (16 bytes):
     * [0]: Header (0x5A)
     * [1]: Type (0x01)
     * [2-5]: Latitude (float, little-endian)
     * [6-9]: Longitude (float, little-endian)
     * [10-13]: Altitude (float, little-endian)
     * [14-15]: Speed * 100 (uint16, little-endian)
     */
    if (len < 16U) {
        return;
    }

    (void)memcpy(&zwift_data.lat, &data[2], sizeof(float));
    (void)memcpy(&zwift_data.lon, &data[6], sizeof(float));
    (void)memcpy(&zwift_data.alt, &data[10], sizeof(float));

    uint16_t speed_raw;
    (void)memcpy(&speed_raw, &data[14], sizeof(uint16_t));
    zwift_data.speed = (float)speed_raw / 100.0f;

    zwift_data.timestamp = k_uptime_get_32();
    zwift_data.valid = true;

    LOG_DBG("Zwift pos: %.5f,%.5f alt=%.0f spd=%.1f",
            (double)zwift_data.lat, (double)zwift_data.lon,
            (double)zwift_data.alt, (double)zwift_data.speed);
}

/**
 * @brief Parse power packet from Zwift
 */
static void parse_power_packet(const uint8_t *data, uint16_t len)
{
    /* Expected format (6 bytes):
     * [0]: Header (0x5A)
     * [1]: Type (0x02)
     * [2-3]: Power (uint16, little-endian)
     * [4-7]: Distance (float, little-endian)
     */
    if (len < 6U) {
        return;
    }

    (void)memcpy(&zwift_data.power, &data[2], sizeof(uint16_t));

    if (len >= 8U) {
        (void)memcpy(&zwift_data.distance, &data[4], sizeof(float));
    }

    zwift_data.timestamp = k_uptime_get_32();

    LOG_DBG("Zwift power: %u W, dist=%.0f m",
            zwift_data.power, (double)zwift_data.distance);
}

/**
 * @brief Parse gradient packet from Zwift
 */
static void parse_gradient_packet(const uint8_t *data, uint16_t len)
{
    /* Expected format (6 bytes):
     * [0]: Header (0x5A)
     * [1]: Type (0x03)
     * [2-5]: Gradient (float, little-endian)
     */
    if (len < 6U) {
        return;
    }

    (void)memcpy(&zwift_data.gradient, &data[2], sizeof(float));
    zwift_data.timestamp = k_uptime_get_32();

    LOG_DBG("Zwift gradient: %.1f%%", (double)zwift_data.gradient);
}

/**
 * @brief Update attitude module with Zwift data
 */
static void update_attitude_from_zwift(void)
{
    if (!zwift_data.valid) {
        return;
    }

    /* Create location data from Zwift simulation */
    loc_data_t loc = {
        .lat = zwift_data.lat,
        .lon = zwift_data.lon,
        .alt = zwift_data.alt,
        .speed = zwift_data.speed,
        .course = 0.0f,
        .timestamp = zwift_data.timestamp
    };

    /* Update attitude with simulated position */
    (void)attitude_update_gps(&loc);

    /* Note: Power data comes from FEC trainer, not Zwift simulation */
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t zwift_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    (void)memset(&zwift_data, 0, sizeof(zwift_data));
    is_active = false;
    target_gradient = 0.0f;
    target_power = 0U;

    is_initialized = true;
    LOG_INF("Zwift mode initialized");

    return APP_OK;
}

app_err_t zwift_enter(void)
{
    if (!is_initialized) {
        (void)zwift_init();
    }

    if (is_active) {
        return APP_OK;
    }

    /* Put GPS to standby - we use simulated position */
    (void)gps_mgmt_standby();

    /* Reset attitude for fresh start */
    attitude_reset();

    /* Clear Zwift data */
    (void)memset(&zwift_data, 0, sizeof(zwift_data));

    /* Set mode in boucle */
    (void)boucle_set_mode(APP_MODE_ZWIFT);

    is_active = true;
    LOG_INF("Entered Zwift mode (GPS standby)");

    return APP_OK;
}

app_err_t zwift_exit(void)
{
    if (!is_active) {
        return APP_OK;
    }

    /* Restore GPS */
    (void)gps_mgmt_wake();

    /* Clear mode */
    (void)boucle_set_mode(APP_MODE_CRS);

    is_active = false;
    LOG_INF("Exited Zwift mode (GPS restored)");

    return APP_OK;
}

bool zwift_is_active(void)
{
    return is_active;
}

void zwift_update_from_ble(const uint8_t *data, uint16_t len)
{
    if (!is_active || (data == NULL) || (len < 2U)) {
        return;
    }

    /* Check for Zwift header */
    if (data[0] != ZWIFT_PACKET_HEADER) {
        LOG_WRN("Invalid Zwift packet header: 0x%02X", data[0]);
        return;
    }

    /* Parse based on packet type */
    switch (data[1]) {
    case ZWIFT_PKT_POSITION:
        parse_position_packet(data, len);
        break;

    case ZWIFT_PKT_POWER:
        parse_power_packet(data, len);
        break;

    case ZWIFT_PKT_GRADIENT:
        parse_gradient_packet(data, len);
        /* Update FEC trainer with gradient */
        zwift_set_gradient(zwift_data.gradient);
        break;

    default:
        LOG_WRN("Unknown Zwift packet type: 0x%02X", data[1]);
        break;
    }
}

app_err_t zwift_get_data(zwift_data_t *out_data)
{
    if (out_data == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *out_data = zwift_data;
    return APP_OK;
}

void zwift_process(void)
{
    if (!is_active) {
        return;
    }

    /* Check for data timeout */
    uint32_t now = k_uptime_get_32();
    if ((zwift_data.timestamp > 0U) &&
        ((now - zwift_data.timestamp) > ZWIFT_DATA_TIMEOUT_MS)) {
        zwift_data.valid = false;
        LOG_WRN("Zwift data timeout");
    }

    /* Update attitude from Zwift data */
    if (zwift_data.valid) {
        update_attitude_from_zwift();
    }

    /* Send trainer control commands if needed */
    if (target_gradient != 0.0f) {
        /* Send gradient to FEC trainer */
        int8_t grade = (int8_t)target_gradient;
        if (grade > FEC_MAX_GRADE) {
            grade = FEC_MAX_GRADE;
        } else if (grade < FEC_MIN_GRADE) {
            grade = FEC_MIN_GRADE;
        }
        (void)ble_fec_client_set_grade(grade);
    } else if (target_power > 0U) {
        /* Send target power to FEC trainer */
        (void)ble_fec_client_set_target_power(target_power);
    }
}

void zwift_set_gradient(float gradient)
{
    target_gradient = gradient;
    target_power = 0U; /* Disable power mode when using gradient */

    LOG_DBG("Zwift gradient target: %.1f%%", (double)gradient);
}

void zwift_set_target_power(uint16_t power)
{
    target_power = power;
    target_gradient = 0.0f; /* Disable gradient mode when using power */

    LOG_DBG("Zwift power target: %u W", power);
}
