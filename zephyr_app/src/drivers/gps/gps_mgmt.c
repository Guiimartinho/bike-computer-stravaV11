/**
 * @file gps_mgmt.c
 * @brief GPS Management module implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "drivers/gps_mgmt.h"
#include "drivers/nmea_parser.h"
#include "hal/hal_uart.h"
#include "hal/hal_gpio.h"

LOG_MODULE_REGISTER(gps_mgmt, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** GPS fix timeout in milliseconds */
#define GPS_FIX_TIMEOUT_MS      60000U

/** Minimum satellites for valid fix */
#define GPS_MIN_SATELLITES      4U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current GPS state */
static gps_state_t gps_state = GPS_STATE_OFF;

/** Latest GPS data */
static gps_data_t gps_data;

/** Fix callback */
static gps_fix_callback_t fix_callback;

/** Initialization flag */
static bool is_initialized;

/** Last fix timestamp */
static uint32_t last_fix_time;

/** NMEA data from parser */
static nmea_data_t nmea_data;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief NMEA line received callback
 */
static void nmea_line_callback(const char *line)
{
    if (line == NULL) {
        return;
    }

    /* Parse the NMEA sentence */
    if (nmea_parser_sentence(line, &nmea_data) == APP_OK) {
        /* Update GPS data based on sentence type */
        switch (nmea_data.type) {
        case NMEA_GGA:
            if (nmea_data.fix_valid) {
                gps_data.location.lat = nmea_data.latitude;
                gps_data.location.lon = nmea_data.longitude;
                gps_data.location.alt = nmea_data.altitude;
                gps_data.quality = (gps_fix_quality_t)nmea_data.fix_quality;
                gps_data.satellites = nmea_data.satellites;
                gps_data.hdop = nmea_data.hdop;
                gps_data.fix_valid = true;

                /* Update state based on satellites */
                if (nmea_data.satellites >= GPS_MIN_SATELLITES) {
                    gps_state = GPS_STATE_FIX_3D;
                } else if (nmea_data.satellites >= 3U) {
                    gps_state = GPS_STATE_FIX_2D;
                }

                last_fix_time = k_uptime_get_32();

                LOG_DBG("GGA: %.6f, %.6f, %.1f m, %d sats",
                        (double)gps_data.location.lat,
                        (double)gps_data.location.lon,
                        (double)gps_data.location.alt,
                        gps_data.satellites);
            }
            break;

        case NMEA_RMC:
            if (nmea_data.fix_valid) {
                gps_data.location.lat = nmea_data.latitude;
                gps_data.location.lon = nmea_data.longitude;
                gps_data.location.speed = nmea_data.speed_kmh;
                gps_data.location.course = nmea_data.course;

                /* Update datetime */
                gps_data.datetime.date = (uint32_t)nmea_data.day * 10000U +
                                         (uint32_t)nmea_data.month * 100U +
                                         (nmea_data.year % 100U);
                gps_data.datetime.secj = (uint32_t)nmea_data.hour * 3600U +
                                         (uint32_t)nmea_data.minute * 60U +
                                         nmea_data.second;
                gps_data.datetime.timestamp = k_uptime_get_32();

                gps_data.fix_valid = true;

                LOG_DBG("RMC: %.1f km/h, course %.1f",
                        (double)gps_data.location.speed,
                        (double)gps_data.location.course);
            }
            break;

        case NMEA_GSA:
            gps_data.hdop = nmea_data.hdop;
            gps_data.vdop = nmea_data.vdop;
            gps_data.pdop = nmea_data.pdop;
            break;

        case NMEA_VTG:
            gps_data.location.speed = nmea_data.speed_kmh;
            gps_data.location.course = nmea_data.course;
            break;

        default:
            /* Other sentence types are not processed */
            break;
        }

        /* Update timestamp */
        gps_data.location.timestamp = k_uptime_get_32();

        /* Notify callback if fix is valid */
        if (gps_data.fix_valid && (fix_callback != NULL)) {
            fix_callback(&gps_data);
        }
    }
}

/**
 * @brief Set GPS module to standby via GPIO
 */
static void set_standby_pin(bool standby)
{
    (void)hal_gpio_set(HAL_GPIO_GPS_STDBY, standby);
}

/**
 * @brief Assert GPS reset via GPIO
 */
static void assert_reset(void)
{
    (void)hal_gpio_set(HAL_GPIO_GPS_RESET, true);
    k_msleep(100);
    (void)hal_gpio_set(HAL_GPIO_GPS_RESET, false);
    k_msleep(100);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t gps_mgmt_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Initialize NMEA parser */
    nmea_parser_init();

    /* Register UART callback for NMEA lines */
    app_err_t err = hal_uart_register_line_callback(HAL_UART_GPS, nmea_line_callback);
    if (err != APP_OK) {
        LOG_ERR("Failed to register UART callback");
        return err;
    }

    /* Initialize GPS data */
    (void)memset(&gps_data, 0, sizeof(gps_data));
    (void)memset(&nmea_data, 0, sizeof(nmea_data));

    /* Set initial state */
    gps_state = GPS_STATE_INIT;

    is_initialized = true;
    LOG_INF("GPS management initialized");

    return APP_OK;
}

app_err_t gps_mgmt_start(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Deassert standby */
    set_standby_pin(false);

    /* Enable UART */
    app_err_t err = hal_uart_set_power(HAL_UART_GPS, true);
    if (err != APP_OK) {
        return err;
    }

    gps_state = GPS_STATE_ACQUIRING;
    LOG_INF("GPS started");

    return APP_OK;
}

app_err_t gps_mgmt_stop(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Assert standby */
    set_standby_pin(true);

    /* Disable UART */
    app_err_t err = hal_uart_set_power(HAL_UART_GPS, false);
    if (err != APP_OK) {
        return err;
    }

    gps_state = GPS_STATE_OFF;
    LOG_INF("GPS stopped");

    return APP_OK;
}

app_err_t gps_mgmt_standby(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    set_standby_pin(true);
    gps_state = GPS_STATE_STANDBY;
    LOG_INF("GPS in standby");

    return APP_OK;
}

app_err_t gps_mgmt_wake(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    set_standby_pin(false);

    /* Resume previous state */
    if (gps_data.fix_valid) {
        gps_state = GPS_STATE_FIX_3D;
    } else {
        gps_state = GPS_STATE_ACQUIRING;
    }

    LOG_INF("GPS woken from standby");

    return APP_OK;
}

app_err_t gps_mgmt_reset(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    assert_reset();

    /* Clear data */
    (void)memset(&gps_data, 0, sizeof(gps_data));
    nmea_parser_init();

    gps_state = GPS_STATE_INIT;
    LOG_INF("GPS reset");

    return APP_OK;
}

gps_state_t gps_mgmt_get_state(void)
{
    return gps_state;
}

bool gps_mgmt_has_fix(void)
{
    return gps_data.fix_valid &&
           ((gps_state == GPS_STATE_FIX_2D) || (gps_state == GPS_STATE_FIX_3D));
}

app_err_t gps_mgmt_get_data(gps_data_t *data)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (data == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *data = gps_data;
    return APP_OK;
}

app_err_t gps_mgmt_get_location(loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (loc == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *loc = gps_data.location;
    return APP_OK;
}

app_err_t gps_mgmt_register_callback(gps_fix_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    fix_callback = callback;
    return APP_OK;
}

app_err_t gps_mgmt_set_rate(uint8_t rate_hz)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((rate_hz == 0U) || (rate_hz > 10U)) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Send rate configuration command to GPS module */
    /* This is module-specific - example for MediaTek: */
    char cmd[32];
    uint16_t period_ms = 1000U / rate_hz;

    /* PMTK220: Set NMEA output rate */
    int len = snprintf(cmd, sizeof(cmd), "$PMTK220,%u*", period_ms);
    if (len > 0) {
        uint8_t checksum = nmea_calculate_checksum(&cmd[1]);
        len = snprintf(cmd, sizeof(cmd), "$PMTK220,%u*%02X\r\n", period_ms, checksum);

        if (len > 0) {
            return hal_uart_transmit_str(HAL_UART_GPS, cmd);
        }
    }

    return APP_ERR_INTERNAL;
}

void gps_mgmt_process(void)
{
    if (!is_initialized) {
        return;
    }

    /* Check for fix timeout */
    if ((gps_state == GPS_STATE_FIX_2D) || (gps_state == GPS_STATE_FIX_3D)) {
        uint32_t now = k_uptime_get_32();
        if ((now - last_fix_time) > GPS_FIX_TIMEOUT_MS) {
            LOG_WRN("GPS fix lost (timeout)");
            gps_state = GPS_STATE_ACQUIRING;
            gps_data.fix_valid = false;
        }
    }

    /* Check fix pin status */
    bool fix_pin = gps_mgmt_check_fix_pin();
    if (!fix_pin && (gps_state == GPS_STATE_FIX_3D)) {
        /* Fix indicator went low */
        gps_state = GPS_STATE_ACQUIRING;
    }
}

uint8_t gps_mgmt_get_satellites(void)
{
    return gps_data.satellites;
}

bool gps_mgmt_check_fix_pin(void)
{
    bool state = false;
    (void)hal_gpio_get(HAL_GPIO_GPS_FIX, &state);
    return state;
}
