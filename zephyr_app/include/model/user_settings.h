/**
 * @file user_settings.h
 * @brief User settings storage and management
 * @note Follows MISRA C:2012 guidelines
 *
 * Manages persistent user settings stored in FRAM/NVS including:
 * - ANT+ device IDs (HRM, BSC, FEC, Glasses)
 * - User profile (FTP, Weight)
 * - Magnetometer calibration
 * - Configuration version and CRC validation
 */

#ifndef MODEL_USER_SETTINGS_H_
#define MODEL_USER_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Settings storage address in FRAM/NVS */
#define SETTINGS_STORAGE_ADDRESS    0x0000U

/** Current settings version - increment when structure changes */
#define SETTINGS_VERSION            0x0003U

/** Default FTP value in watts */
#define DEFAULT_FTP                 240U

/** Default weight in hectograms (e.g., 750 = 75.0 kg) */
#define DEFAULT_WEIGHT              750U

/** Default HRM device ID */
#define DEFAULT_HRM_DEVID           0x0D22U

/** Default BSC device ID */
#define DEFAULT_BSC_DEVID           0U

/** Default FEC device ID */
#define DEFAULT_FEC_DEVID           2846U

/** Default Glasses device ID */
#define DEFAULT_GLA_DEVID           0U

/** Magnetometer calibration present marker */
#define MAG_CAL_PRESENT_ID          0xCAFEU

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Magnetometer calibration data
 */
typedef struct {
    int16_t calib[3];       /**< X, Y, Z offset values */
    uint16_t is_present;    /**< Set to MAG_CAL_PRESENT_ID if valid */
} mag_cal_t;

/**
 * @brief User parameters structure
 * @note Union allows byte-level access for CRC calculation
 */
typedef union {
    struct {
        uint16_t hrm_devid;     /**< Heart Rate Monitor device ID */
        uint16_t bsc_devid;     /**< Bike Speed/Cadence device ID */
        uint16_t gla_devid;     /**< Glasses device ID */
        uint16_t fec_devid;     /**< FE-C (trainer) device ID */
        uint16_t ftp;           /**< Functional Threshold Power (watts) */
        uint16_t weight;        /**< Weight in hectograms */
        uint16_t version;       /**< Settings version number */
        mag_cal_t mag_cal;      /**< Magnetometer calibration */
        uint16_t crc;           /**< CRC-8 checksum (stored as uint16 for alignment) */
    };
    uint8_t flat_params[1];     /**< Byte access for CRC calculation */
} user_params_t;

/**
 * @brief User settings manager context
 */
typedef struct {
    user_params_t params;       /**< Current parameters */
    bool is_loaded;             /**< True if params loaded from storage */
    bool is_dirty;              /**< True if params modified but not saved */
} user_settings_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize user settings manager
 * @param settings Pointer to settings structure
 * @return APP_OK on success, error code otherwise
 */
app_err_t user_settings_init(user_settings_t *settings);

/**
 * @brief Load settings from persistent storage
 * @param settings Pointer to settings structure
 * @return APP_OK on success, APP_ERR_CHECKSUM if CRC invalid
 */
app_err_t user_settings_load(user_settings_t *settings);

/**
 * @brief Save settings to persistent storage
 * @param settings Pointer to settings structure
 * @return APP_OK on success, error code otherwise
 */
app_err_t user_settings_save(user_settings_t *settings);

/**
 * @brief Check if settings are valid (version and CRC)
 * @param settings Pointer to settings structure
 * @return true if valid, false otherwise
 */
bool user_settings_is_valid(const user_settings_t *settings);

/**
 * @brief Reset settings to factory defaults
 * @param settings Pointer to settings structure
 * @return APP_OK on success
 */
app_err_t user_settings_reset(user_settings_t *settings);

/**
 * @brief Ensure valid settings (load or reset if invalid)
 * @param settings Pointer to settings structure
 * @return APP_OK on success
 */
app_err_t user_settings_enforce(user_settings_t *settings);

/**
 * @brief Get HRM device ID
 * @param settings Pointer to settings structure
 * @return Device ID or 0 if settings NULL
 */
uint16_t user_settings_get_hrm_devid(const user_settings_t *settings);

/**
 * @brief Set HRM device ID
 * @param settings Pointer to settings structure
 * @param devid New device ID
 */
void user_settings_set_hrm_devid(user_settings_t *settings, uint16_t devid);

/**
 * @brief Get BSC device ID
 * @param settings Pointer to settings structure
 * @return Device ID or 0 if settings NULL
 */
uint16_t user_settings_get_bsc_devid(const user_settings_t *settings);

/**
 * @brief Set BSC device ID
 * @param settings Pointer to settings structure
 * @param devid New device ID
 */
void user_settings_set_bsc_devid(user_settings_t *settings, uint16_t devid);

/**
 * @brief Get FEC device ID
 * @param settings Pointer to settings structure
 * @return Device ID or 0 if settings NULL
 */
uint16_t user_settings_get_fec_devid(const user_settings_t *settings);

/**
 * @brief Set FEC device ID
 * @param settings Pointer to settings structure
 * @param devid New device ID
 */
void user_settings_set_fec_devid(user_settings_t *settings, uint16_t devid);

/**
 * @brief Get FTP value
 * @param settings Pointer to settings structure
 * @return FTP in watts or 0 if settings NULL
 */
uint16_t user_settings_get_ftp(const user_settings_t *settings);

/**
 * @brief Set FTP value
 * @param settings Pointer to settings structure
 * @param ftp New FTP value in watts
 */
void user_settings_set_ftp(user_settings_t *settings, uint16_t ftp);

/**
 * @brief Get weight value
 * @param settings Pointer to settings structure
 * @return Weight in hectograms or 0 if settings NULL
 */
uint16_t user_settings_get_weight(const user_settings_t *settings);

/**
 * @brief Set weight value
 * @param settings Pointer to settings structure
 * @param weight New weight in hectograms
 */
void user_settings_set_weight(user_settings_t *settings, uint16_t weight);

/**
 * @brief Get magnetometer calibration
 * @param settings Pointer to settings structure
 * @return Pointer to mag_cal structure or NULL
 */
const mag_cal_t *user_settings_get_mag_cal(const user_settings_t *settings);

/**
 * @brief Set magnetometer calibration
 * @param settings Pointer to settings structure
 * @param cal Pointer to calibration data
 */
void user_settings_set_mag_cal(user_settings_t *settings, const mag_cal_t *cal);

/**
 * @brief Check if magnetometer is calibrated
 * @param settings Pointer to settings structure
 * @return true if calibration data is present
 */
bool user_settings_is_mag_calibrated(const user_settings_t *settings);

/**
 * @brief Get global user settings instance
 * @return Pointer to global settings
 */
user_settings_t *user_settings_get_global(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_USER_SETTINGS_H_ */
