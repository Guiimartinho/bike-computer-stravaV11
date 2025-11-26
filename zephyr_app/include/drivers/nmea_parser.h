/**
 * @file nmea_parser.h
 * @brief NMEA 0183 sentence parser for stravaV10
 *
 * Parses NMEA sentences from GPS modules.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_NMEA_PARSER_H
#define DRIVERS_NMEA_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** Maximum NMEA sentence length */
#define NMEA_MAX_SENTENCE_LEN   83U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** NMEA sentence types */
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GGA,       /**< Global Positioning System Fix Data */
    NMEA_RMC,       /**< Recommended Minimum Navigation Information */
    NMEA_GSA,       /**< GPS DOP and active satellites */
    NMEA_GSV,       /**< Satellites in view */
    NMEA_VTG,       /**< Track made good and ground speed */
    NMEA_GLL,       /**< Geographic Position - Latitude/Longitude */
    NMEA_ZDA        /**< Time and Date */
} nmea_type_t;

/** Maximum satellites to track */
#define NMEA_MAX_SATELLITES     12U

/** Satellite information structure (from GSV) */
typedef struct {
    uint8_t prn;                /**< Satellite PRN number */
    uint8_t elevation;          /**< Elevation in degrees (0-90) */
    uint16_t azimuth;           /**< Azimuth in degrees (0-359) */
    uint8_t snr;                /**< Signal-to-Noise Ratio (0-99 dB-Hz) */
    bool in_use;                /**< Satellite used in fix (from GSA) */
} nmea_satellite_t;

/** NMEA parse result */
typedef struct {
    nmea_type_t type;           /**< Sentence type */
    bool valid;                 /**< Parse success flag */
    bool fix_valid;             /**< Fix validity (from sentence) */

    /* Position data (GGA, RMC, GLL) */
    float latitude;             /**< Latitude in degrees */
    float longitude;            /**< Longitude in degrees */
    float altitude;             /**< Altitude in meters (GGA) */
    float speed_knots;          /**< Speed in knots (RMC, VTG) */
    float speed_kmh;            /**< Speed in km/h (VTG) */
    float course;               /**< Course over ground in degrees */

    /* Quality data (GGA, GSA) */
    uint8_t fix_quality;        /**< Fix quality indicator */
    uint8_t satellites;         /**< Number of satellites used in fix */
    float hdop;                 /**< Horizontal DOP */
    float vdop;                 /**< Vertical DOP */
    float pdop;                 /**< Position DOP */

    /* Time data */
    uint8_t hour;               /**< UTC hour */
    uint8_t minute;             /**< UTC minute */
    uint8_t second;             /**< UTC second */
    uint16_t millisecond;       /**< Milliseconds */

    /* Date data (RMC, ZDA) */
    uint8_t day;                /**< Day of month */
    uint8_t month;              /**< Month (1-12) */
    uint16_t year;              /**< Year */

    /* GSV data - satellites in view */
    uint8_t sats_in_view;       /**< Total satellites in view */
    uint8_t gsv_sat_count;      /**< Satellites with info parsed */
} nmea_data_t;

/** Extended satellite data (accumulated from multiple GSV sentences) */
typedef struct {
    nmea_satellite_t sats[NMEA_MAX_SATELLITES];  /**< Satellite info */
    uint8_t count;                               /**< Number of valid entries */
    uint8_t sats_in_view;                        /**< Total sats in view */
    uint8_t prns_in_use[12];                     /**< PRNs used in fix (from GSA) */
    uint8_t in_use_count;                        /**< Number of sats in use */
} nmea_satellites_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize NMEA parser
 */
void nmea_parser_init(void);

/**
 * @brief Parse a single character
 * @param c Character to parse
 * @return true if a complete sentence was parsed
 */
bool nmea_parser_char(char c);

/**
 * @brief Parse a complete NMEA sentence
 * @param sentence Null-terminated NMEA sentence
 * @param data Pointer to store parsed data
 * @return APP_OK on success, error code otherwise
 */
app_err_t nmea_parser_sentence(const char *sentence, nmea_data_t *data);

/**
 * @brief Get last parsed data
 * @param data Pointer to store parsed data
 * @return APP_OK on success, error code otherwise
 */
app_err_t nmea_parser_get_data(nmea_data_t *data);

/**
 * @brief Check if position is valid
 * @return true if valid position available
 */
bool nmea_parser_has_position(void);

/**
 * @brief Check if time is valid
 * @return true if valid time available
 */
bool nmea_parser_has_time(void);

/**
 * @brief Calculate checksum of NMEA sentence
 * @param sentence NMEA sentence (without $ and checksum)
 * @return Calculated checksum
 */
uint8_t nmea_calculate_checksum(const char *sentence);

/**
 * @brief Verify checksum of complete NMEA sentence
 * @param sentence Complete NMEA sentence with $, *, and checksum
 * @return true if checksum is valid
 */
bool nmea_verify_checksum(const char *sentence);

/**
 * @brief Get satellite information
 * @param sats Pointer to store satellite data
 * @return APP_OK on success
 */
app_err_t nmea_parser_get_satellites(nmea_satellites_t *sats);

/**
 * @brief Get number of satellites in view
 * @return Number of satellites in view
 */
uint8_t nmea_parser_get_sats_in_view(void);

/**
 * @brief Get average SNR of satellites in use
 * @return Average SNR in dB-Hz
 */
uint8_t nmea_parser_get_avg_snr(void);

/**
 * @brief Reset satellite tracking data
 */
void nmea_parser_reset_satellites(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_NMEA_PARSER_H */
