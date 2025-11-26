/**
 * @file fxos.c
 * @brief FXOS8700 Accelerometer/Magnetometer driver implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "drivers/fxos.h"

LOG_MODULE_REGISTER(fxos, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Conversion factor from sensor units to mg */
#define ACCEL_SCALE_MG      0.244f

/** Conversion factor from sensor units to uT */
#define MAG_SCALE_UT        0.1f

/** PI constant */
#define PI_F                3.14159265358979323846f

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

static const struct device *fxos_dev = DEVICE_DT_GET(DT_NODELABEL(fxos));

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Latest measurement data */
static fxos_data_t latest_data;

/** Calibration data */
static fxos_calib_t calibration = {
    .mag_offset_x = 0,
    .mag_offset_y = 0,
    .mag_offset_z = 0,
    .mag_scale_x = 1.0f,
    .mag_scale_y = 1.0f,
    .mag_scale_z = 1.0f,
    .calibrated = false,
};

/** Calibration state */
static struct {
    bool active;
    int16_t mag_min[3];
    int16_t mag_max[3];
    uint32_t sample_count;
} calib_state;

/** Data ready flag */
static volatile bool data_ready;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate tilt-compensated heading
 */
static void calculate_orientation(void)
{
    /* Apply calibration to magnetometer */
    float mx = (float)(latest_data.mag.x - calibration.mag_offset_x) * calibration.mag_scale_x;
    float my = (float)(latest_data.mag.y - calibration.mag_offset_y) * calibration.mag_scale_y;
    float mz = (float)(latest_data.mag.z - calibration.mag_offset_z) * calibration.mag_scale_z;

    /* Get accelerometer values */
    float ax = (float)latest_data.accel.x;
    float ay = (float)latest_data.accel.y;
    float az = (float)latest_data.accel.z;

    /* Calculate roll and pitch from accelerometer */
    latest_data.roll = atan2f(ay, az);
    latest_data.pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    /* Tilt-compensated magnetic heading */
    float cos_roll = cosf(latest_data.roll);
    float sin_roll = sinf(latest_data.roll);
    float cos_pitch = cosf(latest_data.pitch);
    float sin_pitch = sinf(latest_data.pitch);

    float mx_comp = mx * cos_pitch + mz * sin_pitch;
    float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    /* Calculate yaw (heading) */
    latest_data.yaw = atan2f(-my_comp, mx_comp);

    /* Normalize yaw to 0-2*PI */
    if (latest_data.yaw < 0.0f) {
        latest_data.yaw += 2.0f * PI_F;
    }
}

/**
 * @brief Update calibration with new sample
 */
static void update_calibration_sample(void)
{
    if (latest_data.mag.x < calib_state.mag_min[0]) {
        calib_state.mag_min[0] = latest_data.mag.x;
    }
    if (latest_data.mag.x > calib_state.mag_max[0]) {
        calib_state.mag_max[0] = latest_data.mag.x;
    }

    if (latest_data.mag.y < calib_state.mag_min[1]) {
        calib_state.mag_min[1] = latest_data.mag.y;
    }
    if (latest_data.mag.y > calib_state.mag_max[1]) {
        calib_state.mag_max[1] = latest_data.mag.y;
    }

    if (latest_data.mag.z < calib_state.mag_min[2]) {
        calib_state.mag_min[2] = latest_data.mag.z;
    }
    if (latest_data.mag.z > calib_state.mag_max[2]) {
        calib_state.mag_max[2] = latest_data.mag.z;
    }

    calib_state.sample_count++;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t fxos_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    if (!device_is_ready(fxos_dev)) {
        LOG_ERR("FXOS8700 device not ready");
        return APP_ERR_NOT_INIT;
    }

    /* Clear data structures */
    (void)memset(&latest_data, 0, sizeof(latest_data));
    (void)memset(&calib_state, 0, sizeof(calib_state));

    /* Initialize calibration min/max */
    calib_state.mag_min[0] = INT16_MAX;
    calib_state.mag_min[1] = INT16_MAX;
    calib_state.mag_min[2] = INT16_MAX;
    calib_state.mag_max[0] = INT16_MIN;
    calib_state.mag_max[1] = INT16_MIN;
    calib_state.mag_max[2] = INT16_MIN;

    is_initialized = true;
    LOG_INF("FXOS8700 initialized");

    return APP_OK;
}

app_err_t fxos_trigger(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Fetch all channels */
    int ret = sensor_sample_fetch(fxos_dev);
    if (ret < 0) {
        LOG_ERR("Failed to fetch FXOS sample: %d", ret);
        return APP_ERR_IO;
    }

    /* Read accelerometer */
    struct sensor_value accel[3];
    ret = sensor_channel_get(fxos_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (ret < 0) {
        LOG_ERR("Failed to get accel data: %d", ret);
        return APP_ERR_IO;
    }

    /* Convert accelerometer to mg */
    latest_data.accel.x = (int16_t)(sensor_value_to_double(&accel[0]) * 1000.0 / 9.81);
    latest_data.accel.y = (int16_t)(sensor_value_to_double(&accel[1]) * 1000.0 / 9.81);
    latest_data.accel.z = (int16_t)(sensor_value_to_double(&accel[2]) * 1000.0 / 9.81);

    /* Read magnetometer */
    struct sensor_value mag[3];
    ret = sensor_channel_get(fxos_dev, SENSOR_CHAN_MAGN_XYZ, mag);
    if (ret < 0) {
        LOG_WRN("Failed to get mag data: %d", ret);
        /* Magnetometer may not be available on all devices */
    } else {
        latest_data.mag.x = (int16_t)(sensor_value_to_double(&mag[0]) * 10.0);
        latest_data.mag.y = (int16_t)(sensor_value_to_double(&mag[1]) * 10.0);
        latest_data.mag.z = (int16_t)(sensor_value_to_double(&mag[2]) * 10.0);
    }

    /* Update calibration if active */
    if (calib_state.active) {
        update_calibration_sample();
    }

    /* Calculate orientation */
    calculate_orientation();

    /* Update timestamp and validity */
    latest_data.timestamp = k_uptime_get_32();
    latest_data.valid = true;
    data_ready = true;

    return APP_OK;
}

app_err_t fxos_read(fxos_data_t *data)
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

void fxos_calibration_start(void)
{
    /* Reset min/max values */
    calib_state.mag_min[0] = INT16_MAX;
    calib_state.mag_min[1] = INT16_MAX;
    calib_state.mag_min[2] = INT16_MAX;
    calib_state.mag_max[0] = INT16_MIN;
    calib_state.mag_max[1] = INT16_MIN;
    calib_state.mag_max[2] = INT16_MIN;
    calib_state.sample_count = 0;

    calib_state.active = true;
    LOG_INF("Magnetometer calibration started");
}

app_err_t fxos_calibration_stop(void)
{
    if (!calib_state.active) {
        return APP_ERR_NOT_INIT;
    }

    calib_state.active = false;

    if (calib_state.sample_count < 100U) {
        LOG_WRN("Insufficient calibration samples: %u", calib_state.sample_count);
        return APP_ERR_INVALID_PARAM;
    }

    /* Calculate hard iron offsets (center of sphere) */
    calibration.mag_offset_x = (calib_state.mag_min[0] + calib_state.mag_max[0]) / 2;
    calibration.mag_offset_y = (calib_state.mag_min[1] + calib_state.mag_max[1]) / 2;
    calibration.mag_offset_z = (calib_state.mag_min[2] + calib_state.mag_max[2]) / 2;

    /* Calculate soft iron scales (ellipsoid to sphere) */
    float range_x = (float)(calib_state.mag_max[0] - calib_state.mag_min[0]);
    float range_y = (float)(calib_state.mag_max[1] - calib_state.mag_min[1]);
    float range_z = (float)(calib_state.mag_max[2] - calib_state.mag_min[2]);

    float avg_range = (range_x + range_y + range_z) / 3.0f;

    if ((range_x > 0.0f) && (range_y > 0.0f) && (range_z > 0.0f)) {
        calibration.mag_scale_x = avg_range / range_x;
        calibration.mag_scale_y = avg_range / range_y;
        calibration.mag_scale_z = avg_range / range_z;
        calibration.calibrated = true;

        LOG_INF("Calibration complete: offset(%d,%d,%d) scale(%.2f,%.2f,%.2f)",
                calibration.mag_offset_x, calibration.mag_offset_y, calibration.mag_offset_z,
                (double)calibration.mag_scale_x, (double)calibration.mag_scale_y,
                (double)calibration.mag_scale_z);
    } else {
        LOG_ERR("Invalid calibration ranges");
        return APP_ERR_INVALID_PARAM;
    }

    return APP_OK;
}

app_err_t fxos_get_calibration(fxos_calib_t *calib)
{
    if (calib == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *calib = calibration;
    return APP_OK;
}

app_err_t fxos_set_calibration(const fxos_calib_t *calib)
{
    if (calib == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    calibration = *calib;
    return APP_OK;
}

app_err_t fxos_get_yaw(float *yaw)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (yaw == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *yaw = latest_data.yaw;
    return APP_OK;
}

app_err_t fxos_get_pitch(float *pitch)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (pitch == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *pitch = latest_data.pitch;
    return APP_OK;
}

app_err_t fxos_get_roll(float *roll)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (roll == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *roll = latest_data.roll;
    return APP_OK;
}

app_err_t fxos_sleep(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    return APP_OK;
}

app_err_t fxos_wake(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    return APP_OK;
}

void fxos_process(void)
{
    /* Periodic processing - currently empty */
}

bool fxos_is_data_ready(void)
{
    return data_ready;
}
