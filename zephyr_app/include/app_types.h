/**
 * @file app_types.h
 * @brief Common types and definitions for stravaV10 application
 *
 * This file contains all common type definitions, constants, and macros
 * used throughout the application. Follows MISRA C:2012 guidelines.
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Version Information
 * ========================================================================== */

#define APP_VERSION_MAJOR    2U
#define APP_VERSION_MINOR    0U
#define APP_VERSION_PATCH    0U

/* ==========================================================================
 * Common Constants
 * ========================================================================== */

#define APP_NAME             "stravaV10"
#define APP_NAME_LEN         9U

/** Maximum number of segments that can be stored */
#define MAX_SEGMENTS         500U

/** Maximum number of points in history */
#define MAX_HISTORY_POINTS   50U

/** Maximum number of parcours (routes) */
#define MAX_PARCOURS         10U

/** GPS history size for position calculations */
#define HISTO_POINT_SIZE     15U

/** Attitude buffer size for logging */
#define ATT_BUFFER_NB_ELEM   20U

/* ==========================================================================
 * Error Codes
 * ========================================================================== */

typedef enum {
    APP_OK = 0,
    APP_ERR_INVALID_PARAM = -1,
    APP_ERR_NO_MEM = -2,
    APP_ERR_TIMEOUT = -3,
    APP_ERR_BUSY = -4,
    APP_ERR_NOT_INIT = -5,
    APP_ERR_ALREADY_INIT = -6,
    APP_ERR_NOT_FOUND = -7,
    APP_ERR_IO = -8,
    APP_ERR_OVERFLOW = -9,
    APP_ERR_CHECKSUM = -10,
    APP_ERR_INTERNAL = -99
} app_err_t;

/* ==========================================================================
 * Location Types
 * ========================================================================== */

/** Location source enumeration */
typedef enum {
    LOC_SOURCE_NONE = 0,
    LOC_SOURCE_GPS,
    LOC_SOURCE_BLE_LNS,
    LOC_SOURCE_SIM
} loc_source_t;

/** Location data structure */
typedef struct {
    float lat;          /**< Latitude in degrees */
    float lon;          /**< Longitude in degrees */
    float alt;          /**< Altitude in meters */
    float speed;        /**< Speed in km/h */
    float course;       /**< Course in degrees */
    uint32_t timestamp; /**< Timestamp in milliseconds */
} loc_data_t;

/** Date/Time structure */
typedef struct {
    uint32_t date;      /**< Date as DDMMYY */
    uint32_t secj;      /**< Seconds of day */
    uint32_t timestamp; /**< System timestamp when received */
} date_data_t;

/* ==========================================================================
 * Attitude Types
 * ========================================================================== */

/** Main attitude structure - holds current state */
typedef struct {
    loc_data_t loc;         /**< Current location */
    date_data_t date;       /**< Current date/time */
    float dist;             /**< Total distance in meters */
    float climb;            /**< Total climb in meters */
    float vit_asc;          /**< Vertical speed in m/s */
    int8_t slope;           /**< Current slope in percent */
    uint16_t pwr;           /**< Estimated power in watts */
    uint16_t next;          /**< Distance to next segment in meters */
    uint16_t nbpts;         /**< Number of GPS points received */
    uint16_t nbsec_act;     /**< Active seconds (speed > threshold) */
    uint8_t nbact;          /**< Number of active segments */
    uint8_t pr;             /**< Personal record indicator */
} attitude_t;

/* ==========================================================================
 * Segment Types
 * ========================================================================== */

/** Segment status enumeration */
typedef enum {
    SEG_OFF = 0,        /**< Segment inactive */
    SEG_START = 1,      /**< Segment just activated */
    SEG_ON = 2,         /**< Segment active */
    SEG_FIN = -5        /**< Segment finished (countdown to removal) */
} seg_status_t;

/** Segment data structure */
typedef struct {
    char name[13];          /**< Segment name (8.3 format) */
    uint16_t num_points;    /**< Number of points in segment */
    float total_time;       /**< Reference time in seconds */
    float total_elev;       /**< Total elevation gain */
    float cur_time;         /**< Current time on segment */
    float advance;          /**< Time advantage (+) or behind (-) */
    float pct_dist;         /**< Progress percentage (0-1) */
    float pct_elev;         /**< Elevation progress percentage */
    seg_status_t status;    /**< Current status */
    int8_t score;           /**< Display priority score */
} segment_t;

/* ==========================================================================
 * Sensor Types
 * ========================================================================== */

/** Heart Rate Monitor info */
typedef struct {
    uint8_t bpm;            /**< Beats per minute */
    uint16_t rr_interval;   /**< R-R interval in ms */
    uint32_t timestamp;     /**< Last update timestamp */
    bool connected;         /**< Connection status */
} hrm_info_t;

/** Bike Speed/Cadence info */
typedef struct {
    uint16_t speed;         /**< Speed in 0.01 km/h */
    uint8_t cadence;        /**< Cadence in RPM */
    uint32_t timestamp;     /**< Last update timestamp */
    bool connected;         /**< Connection status */
} bsc_info_t;

/** FE-C (Smart Trainer) info */
typedef struct {
    uint16_t power;         /**< Instantaneous power in watts */
    uint16_t el_time;       /**< Elapsed time in seconds */
    uint8_t cadence;        /**< Cadence in RPM */
    int8_t grade;           /**< Current grade in percent */
    bool connected;         /**< Connection status */
} fec_info_t;

/* ==========================================================================
 * Button/Input Types
 * ========================================================================== */

/** Button event enumeration */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_LEFT,
    BTN_EVENT_CENTER,
    BTN_EVENT_RIGHT,
    BTN_EVENT_LONG_LEFT,
    BTN_EVENT_LONG_CENTER,
    BTN_EVENT_LONG_RIGHT
} btn_event_t;

/* ==========================================================================
 * Display/Vue Types
 * ========================================================================== */

/** Display mode enumeration */
typedef enum {
    VUE_MODE_CRS = 0,       /**< Outdoor cycling mode */
    VUE_MODE_FEC,           /**< Indoor trainer mode */
    VUE_MODE_PRC,           /**< Parcours (route) mode */
    VUE_MODE_DEBUG,         /**< Debug information mode */
    VUE_MODE_MENU           /**< Menu mode */
} vue_mode_t;

/** Notification type */
typedef enum {
    NOTIF_TYPE_INFO = 0,
    NOTIF_TYPE_WARNING,
    NOTIF_TYPE_ERROR,
    NOTIF_TYPE_SUCCESS
} notif_type_t;

/** Notification structure */
typedef struct {
    char title[16];         /**< Notification title */
    char message[64];       /**< Notification message */
    uint8_t duration;       /**< Duration in seconds */
    notif_type_t type;      /**< Notification type */
} notification_t;

/* ==========================================================================
 * Global Mode Types
 * ========================================================================== */

/** Application mode enumeration */
typedef enum {
    APP_MODE_INIT = 0,
    APP_MODE_CRS,           /**< Outdoor cycling */
    APP_MODE_FEC,           /**< Indoor trainer */
    APP_MODE_PRC,           /**< Following a parcours */
    APP_MODE_ZWIFT,         /**< Zwift simulation mode */
    APP_MODE_MSC            /**< USB Mass Storage mode */
} app_mode_t;

/* ==========================================================================
 * Utility Macros
 * ========================================================================== */

/** Get minimum of two values */
#define MIN(a, b)           (((a) < (b)) ? (a) : (b))

/** Get maximum of two values */
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))

/** Clamp value between min and max */
#define CLAMP(val, min, max) (MIN(MAX((val), (min)), (max)))

/** Get array element count */
#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))

/** Check if value is within range [min, max] */
#define IN_RANGE(val, min, max) (((val) >= (min)) && ((val) <= (max)))

/** Convert km/h to m/s */
#define KMH_TO_MS(x)        ((x) / 3.6f)

/** Convert m/s to km/h */
#define MS_TO_KMH(x)        ((x) * 3.6f)

/** Degrees to radians */
#define DEG_TO_RAD(x)       ((x) * 0.017453292519943295f)

/** Radians to degrees */
#define RAD_TO_DEG(x)       ((x) * 57.29577951308232f)

#ifdef __cplusplus
}
#endif

#endif /* APP_TYPES_H */
