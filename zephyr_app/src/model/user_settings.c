/**
 * @file user_settings.c
 * @brief User settings storage and management implementation
 *
 * Based on original UserSettings.cpp implementation.
 * Uses Zephyr NVS (Non-Volatile Storage) for persistence.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "model/user_settings.h"

LOG_MODULE_REGISTER(user_settings, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Settings key for NVS storage */
#define SETTINGS_KEY    "strava/user"

/** Size of data for CRC calculation (all fields except CRC) */
#define CRC_DATA_SIZE   (sizeof(user_params_t) - sizeof(uint16_t))

/* ==========================================================================
 * Global Instance
 * ========================================================================== */

/** Global user settings instance */
static user_settings_t g_user_settings;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate CRC-8 of data array (from original)
 * @param data Pointer to data
 * @param len Length of data
 * @return CRC-8 value
 */
static uint8_t calculate_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0U;

    for (uint16_t i = 0U; i < len; i++) {
        uint8_t inbyte = data[i];

        for (uint8_t j = 0U; j < 8U; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01U;
            crc >>= 1U;

            if (mix != 0U) {
                crc ^= 0x8CU;
            }

            inbyte >>= 1U;
        }
    }

    return crc;
}

/**
 * @brief Update CRC in settings
 * @param settings Pointer to settings structure
 */
static void update_crc(user_settings_t *settings)
{
    settings->params.crc = (uint16_t)calculate_crc8(
        settings->params.flat_params,
        CRC_DATA_SIZE
    );
}

/**
 * @brief Verify CRC of settings
 * @param settings Pointer to settings structure
 * @return true if CRC matches
 */
static bool verify_crc(const user_settings_t *settings)
{
    uint8_t calculated = calculate_crc8(
        settings->params.flat_params,
        CRC_DATA_SIZE
    );

    return (calculated == (uint8_t)settings->params.crc);
}

/* ==========================================================================
 * Settings Subsystem Handlers (Zephyr)
 * ========================================================================== */

#if defined(CONFIG_SETTINGS)

static int settings_set_handler(const char *name, size_t len,
                                settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "params", &next) && !next) {
        if (len != sizeof(user_params_t)) {
            LOG_WRN("Invalid settings size: %zu vs %zu", len, sizeof(user_params_t));
            return -EINVAL;
        }

        rc = read_cb(cb_arg, &g_user_settings.params, sizeof(user_params_t));
        if (rc >= 0) {
            g_user_settings.is_loaded = true;
            LOG_INF("Settings loaded from NVS");
            return 0;
        }

        return rc;
    }

    return -ENOENT;
}

static int settings_export_handler(int (*cb)(const char *name,
                                             const void *value, size_t val_len))
{
    return cb(SETTINGS_KEY "/params", &g_user_settings.params, sizeof(user_params_t));
}

SETTINGS_STATIC_HANDLER_DEFINE(user_settings, SETTINGS_KEY,
                               NULL, settings_set_handler,
                               NULL, settings_export_handler);

#endif /* CONFIG_SETTINGS */

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t user_settings_init(user_settings_t *settings)
{
    if (settings == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    (void)memset(settings, 0, sizeof(user_settings_t));

#if defined(CONFIG_SETTINGS)
    int rc = settings_subsys_init();
    if (rc != 0) {
        LOG_ERR("Settings subsystem init failed: %d", rc);
        return APP_ERR_INTERNAL;
    }
#endif

    LOG_INF("User settings initialized");

    return APP_OK;
}

app_err_t user_settings_load(user_settings_t *settings)
{
    if (settings == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

#if defined(CONFIG_SETTINGS)
    int rc = settings_load_subtree(SETTINGS_KEY);
    if (rc != 0) {
        LOG_WRN("Failed to load settings: %d", rc);
        return APP_ERR_IO;
    }

    /* Copy from global if this isn't the global instance */
    if (settings != &g_user_settings) {
        (void)memcpy(&settings->params, &g_user_settings.params, sizeof(user_params_t));
        settings->is_loaded = g_user_settings.is_loaded;
    }

    if (!settings->is_loaded) {
        LOG_WRN("No settings found in storage");
        return APP_ERR_NOT_FOUND;
    }

    /* Verify CRC */
    if (!verify_crc(settings)) {
        LOG_ERR("Settings CRC mismatch!");
        return APP_ERR_CHECKSUM;
    }

    LOG_INF("Settings loaded successfully (v%u)", settings->params.version);
#else
    LOG_WRN("Settings subsystem not enabled");
    return APP_ERR_NOT_INIT;
#endif

    return APP_OK;
}

app_err_t user_settings_save(user_settings_t *settings)
{
    if (settings == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Update CRC before saving */
    update_crc(settings);

#if defined(CONFIG_SETTINGS)
    /* Copy to global if this isn't the global instance */
    if (settings != &g_user_settings) {
        (void)memcpy(&g_user_settings.params, &settings->params, sizeof(user_params_t));
        g_user_settings.is_loaded = true;
    }

    int rc = settings_save_one(SETTINGS_KEY "/params",
                               &settings->params, sizeof(user_params_t));
    if (rc != 0) {
        LOG_ERR("Failed to save settings: %d", rc);
        return APP_ERR_IO;
    }

    settings->is_dirty = false;
    LOG_INF("Settings saved (v%u, CRC=0x%02X)",
            settings->params.version, settings->params.crc);
#else
    LOG_WRN("Settings subsystem not enabled");
    return APP_ERR_NOT_INIT;
#endif

    return APP_OK;
}

bool user_settings_is_valid(const user_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    /* Check version */
    if (settings->params.version != SETTINGS_VERSION) {
        LOG_WRN("Settings version mismatch: %u vs %u",
                settings->params.version, SETTINGS_VERSION);
        return false;
    }

    /* Verify CRC */
    if (!verify_crc(settings)) {
        LOG_WRN("Settings CRC invalid");
        return false;
    }

    LOG_INF("Settings valid (v%u)", settings->params.version);
    return true;
}

app_err_t user_settings_reset(user_settings_t *settings)
{
    if (settings == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Clear all fields */
    (void)memset(&settings->params, 0, sizeof(user_params_t));

    /* Set defaults */
    settings->params.version = SETTINGS_VERSION;
    settings->params.ftp = DEFAULT_FTP;
    settings->params.weight = DEFAULT_WEIGHT;
    settings->params.hrm_devid = DEFAULT_HRM_DEVID;
    settings->params.bsc_devid = DEFAULT_BSC_DEVID;
    settings->params.fec_devid = DEFAULT_FEC_DEVID;
    settings->params.gla_devid = DEFAULT_GLA_DEVID;

    /* Clear mag calibration */
    settings->params.mag_cal.is_present = 0U;

    /* Update CRC */
    update_crc(settings);

    settings->is_loaded = true;
    settings->is_dirty = true;

    LOG_WRN("Settings reset to factory defaults");

    return user_settings_save(settings);
}

app_err_t user_settings_enforce(user_settings_t *settings)
{
    if (settings == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    /* Try to load settings */
    app_err_t err = user_settings_load(settings);

    if (err != APP_OK) {
        LOG_WRN("Failed to load settings (err=%d), resetting", err);
        return user_settings_reset(settings);
    }

    /* Validate settings */
    if (!user_settings_is_valid(settings)) {
        LOG_WRN("Settings invalid, resetting");
        return user_settings_reset(settings);
    }

    return APP_OK;
}

/* Getter/Setter implementations */

uint16_t user_settings_get_hrm_devid(const user_settings_t *settings)
{
    if (settings == NULL) {
        return 0U;
    }
    return settings->params.hrm_devid;
}

void user_settings_set_hrm_devid(user_settings_t *settings, uint16_t devid)
{
    if (settings != NULL) {
        settings->params.hrm_devid = devid;
        settings->is_dirty = true;
    }
}

uint16_t user_settings_get_bsc_devid(const user_settings_t *settings)
{
    if (settings == NULL) {
        return 0U;
    }
    return settings->params.bsc_devid;
}

void user_settings_set_bsc_devid(user_settings_t *settings, uint16_t devid)
{
    if (settings != NULL) {
        settings->params.bsc_devid = devid;
        settings->is_dirty = true;
    }
}

uint16_t user_settings_get_fec_devid(const user_settings_t *settings)
{
    if (settings == NULL) {
        return 0U;
    }
    return settings->params.fec_devid;
}

void user_settings_set_fec_devid(user_settings_t *settings, uint16_t devid)
{
    if (settings != NULL) {
        settings->params.fec_devid = devid;
        settings->is_dirty = true;
    }
}

uint16_t user_settings_get_ftp(const user_settings_t *settings)
{
    if (settings == NULL) {
        return 0U;
    }
    return settings->params.ftp;
}

void user_settings_set_ftp(user_settings_t *settings, uint16_t ftp)
{
    if (settings != NULL) {
        settings->params.ftp = ftp;
        settings->is_dirty = true;
    }
}

uint16_t user_settings_get_weight(const user_settings_t *settings)
{
    if (settings == NULL) {
        return 0U;
    }
    return settings->params.weight;
}

void user_settings_set_weight(user_settings_t *settings, uint16_t weight)
{
    if (settings != NULL) {
        settings->params.weight = weight;
        settings->is_dirty = true;
    }
}

const mag_cal_t *user_settings_get_mag_cal(const user_settings_t *settings)
{
    if (settings == NULL) {
        return NULL;
    }
    return &settings->params.mag_cal;
}

void user_settings_set_mag_cal(user_settings_t *settings, const mag_cal_t *cal)
{
    if ((settings != NULL) && (cal != NULL)) {
        (void)memcpy(&settings->params.mag_cal, cal, sizeof(mag_cal_t));
        settings->params.mag_cal.is_present = MAG_CAL_PRESENT_ID;
        settings->is_dirty = true;
    }
}

bool user_settings_is_mag_calibrated(const user_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }
    return (settings->params.mag_cal.is_present == MAG_CAL_PRESENT_ID);
}

user_settings_t *user_settings_get_global(void)
{
    return &g_user_settings;
}
