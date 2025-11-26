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
    uint8_t satellites;         /**< Number of satellites */
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
} nmea_data_t;

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

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_NMEA_PARSER_H */
