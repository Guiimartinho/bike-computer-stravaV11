/**
 * @file baro.c
 * @brief Barometer driver implementation for BME280/BMP280
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "drivers/baro.h"

LOG_MODULE_REGISTER(baro, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Standard sea level pressure in Pa */
#define SEA_LEVEL_PRESSURE_PA   101325.0f

/** Temperature lapse rate in K/m */
#define TEMP_LAPSE_RATE         0.0065f

/** Altitude calculation constant */
#define ALT_EXPONENT            0.1903f

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

static const struct device *baro_dev = DEVICE_DT_GET(DT_NODELABEL(baro));

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Reference pressure for altitude calculation */
static float ref_pressure = SEA_LEVEL_PRESSURE_PA;

/** Latest measurement data */
static baro_data_t latest_data;

/** Data ready flag */
static volatile bool data_ready;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t baro_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    if (!device_is_ready(baro_dev)) {
        LOG_ERR("Barometer device not ready");
        return APP_ERR_NOT_INIT;
    }

    /* Clear data structure */
    (void)memset(&latest_data, 0, sizeof(latest_data));

    is_initialized = true;
    LOG_INF("Barometer initialized");

    return APP_OK;
}

app_err_t baro_trigger(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Trigger a sample fetch */
    int ret = sensor_sample_fetch(baro_dev);
    if (ret < 0) {
        LOG_ERR("Failed to fetch sample: %d", ret);
        return APP_ERR_IO;
    }

    /* Read pressure */
    struct sensor_value pressure;
    ret = sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &pressure);
    if (ret < 0) {
        LOG_ERR("Failed to get pressure: %d", ret);
        return APP_ERR_IO;
    }

    /* Read temperature */
    struct sensor_value temp;
    ret = sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (ret < 0) {
        LOG_ERR("Failed to get temperature: %d", ret);
        return APP_ERR_IO;
    }

    /* Convert to float values */
    /* Pressure is in kPa, convert to Pa */
    latest_data.pressure = (float)sensor_value_to_double(&pressure) * 1000.0f;
    latest_data.temperature = (float)sensor_value_to_double(&temp);

    /* Try to read humidity (BME280 only) */
    struct sensor_value humidity;
    ret = sensor_channel_get(baro_dev, SENSOR_CHAN_HUMIDITY, &humidity);
    if (ret == 0) {
        latest_data.humidity = (float)sensor_value_to_double(&humidity);
    } else {
        latest_data.humidity = 0.0f;
    }

    /* Calculate altitude */
    latest_data.altitude = baro_pressure_to_altitude(latest_data.pressure);

    /* Update timestamp and validity */
    latest_data.timestamp = k_uptime_get_32();
    latest_data.valid = true;
    data_ready = true;

    return APP_OK;
}

app_err_t baro_read(baro_data_t *data)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (data == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *data = latest_data;
    data_ready = false;

    return APP_OK;
}

void baro_set_reference(float ref_press)
{
    if (ref_press > 0.0f) {
        ref_pressure = ref_press;
    }
}

float baro_get_reference(void)
{
    return ref_pressure;
}

float baro_pressure_to_altitude(float pressure)
{
    if (pressure <= 0.0f) {
        return 0.0f;
    }

    /*
     * Barometric formula:
     * h = (T0 / L) * (1 - (P/P0)^(R*L/(g*M)))
     *
     * Simplified for standard atmosphere:
     * h = 44330 * (1 - (P/P0)^0.1903)
     */
    float ratio = pressure / ref_pressure;
    float altitude = 44330.0f * (1.0f - powf(ratio, ALT_EXPONENT));

    return altitude;
}

app_err_t baro_sleep(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Zephyr sensor API handles power management through PM subsystem */
    /* For explicit control, we would need device-specific attributes */

    return APP_OK;
}

app_err_t baro_wake(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    return APP_OK;
}

bool baro_is_data_ready(void)
{
    return data_ready;
}
