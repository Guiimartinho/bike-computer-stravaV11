/**
 * @file nmea_parser.c
 * @brief NMEA 0183 sentence parser implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "drivers/nmea_parser.h"

LOG_MODULE_REGISTER(nmea, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Parser states */
typedef enum {
    STATE_IDLE = 0,
    STATE_SENTENCE,
    STATE_CHECKSUM1,
    STATE_CHECKSUM2
} parser_state_t;

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Parser state */
static parser_state_t state = STATE_IDLE;

/** Sentence buffer */
static char sentence_buf[NMEA_MAX_SENTENCE_LEN];
static uint8_t sentence_idx;

/** Running checksum */
static uint8_t running_checksum;

/** Received checksum */
static uint8_t received_checksum;

/** Last parsed data */
static nmea_data_t last_data;

/** Position valid flag */
static bool position_valid;

/** Time valid flag */
static bool time_valid;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Convert hex character to value
 */
static uint8_t hex_to_val(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return (uint8_t)(c - '0');
    }
    if ((c >= 'A') && (c <= 'F')) {
        return (uint8_t)(c - 'A' + 10);
    }
    if ((c >= 'a') && (c <= 'f')) {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0U;
}

/**
 * @brief Parse latitude/longitude field
 * @param field DDMM.MMMMM or DDDMM.MMMMM format
 * @param dir Direction (N/S or E/W)
 * @return Degrees with sign
 */
static float parse_coordinate(const char *field, char dir)
{
    if ((field == NULL) || (field[0] == '\0')) {
        return 0.0f;
    }

    float raw = (float)atof(field);
    int32_t degrees = (int32_t)(raw / 100.0f);
    float minutes = raw - (float)(degrees * 100);
    float result = (float)degrees + (minutes / 60.0f);

    if ((dir == 'S') || (dir == 'W')) {
        result = -result;
    }

    return result;
}

/**
 * @brief Parse time field (HHMMSS.sss)
 */
static void parse_time(const char *field, nmea_data_t *data)
{
    if ((field == NULL) || (field[0] == '\0') || (data == NULL)) {
        return;
    }

    /* HHMMSS.sss format */
    if (strlen(field) >= 6U) {
        data->hour = (uint8_t)(((field[0] - '0') * 10) + (field[1] - '0'));
        data->minute = (uint8_t)(((field[2] - '0') * 10) + (field[3] - '0'));
        data->second = (uint8_t)(((field[4] - '0') * 10) + (field[5] - '0'));

        if ((strlen(field) > 7U) && (field[6] == '.')) {
            data->millisecond = (uint16_t)(atoi(&field[7]) * 10);
        }
    }
}

/**
 * @brief Parse date field (DDMMYY)
 */
static void parse_date(const char *field, nmea_data_t *data)
{
    if ((field == NULL) || (field[0] == '\0') || (data == NULL)) {
        return;
    }

    if (strlen(field) >= 6U) {
        data->day = (uint8_t)(((field[0] - '0') * 10) + (field[1] - '0'));
        data->month = (uint8_t)(((field[2] - '0') * 10) + (field[3] - '0'));
        data->year = (uint16_t)(2000 + ((field[4] - '0') * 10) + (field[5] - '0'));
    }
}

/**
 * @brief Get next field from sentence
 */
static const char *get_field(const char *sentence, uint8_t field_num)
{
    const char *ptr = sentence;
    uint8_t count = 0U;

    while (*ptr != '\0') {
        if (*ptr == ',') {
            count++;
            if (count == field_num) {
                return ptr + 1;
            }
        }
        ptr++;
    }

    return NULL;
}

/**
 * @brief Copy field to buffer
 */
static void copy_field(const char *sentence, uint8_t field_num, char *buf, size_t buf_len)
{
    const char *start = get_field(sentence, field_num);
    if (start == NULL) {
        buf[0] = '\0';
        return;
    }

    size_t i = 0U;
    while ((start[i] != '\0') && (start[i] != ',') && (start[i] != '*') &&
           (i < (buf_len - 1U))) {
        buf[i] = start[i];
        i++;
    }
    buf[i] = '\0';
}

/**
 * @brief Parse GGA sentence (Fix data)
 */
static app_err_t parse_gga(const char *sentence, nmea_data_t *data)
{
    char field[20];

    data->type = NMEA_GGA;

    /* Field 1: Time */
    copy_field(sentence, 1U, field, sizeof(field));
    parse_time(field, data);

    /* Field 2-3: Latitude */
    copy_field(sentence, 2U, field, sizeof(field));
    char lat_dir = '\0';
    const char *dir = get_field(sentence, 3U);
    if (dir != NULL) {
        lat_dir = *dir;
    }
    data->latitude = parse_coordinate(field, lat_dir);

    /* Field 4-5: Longitude */
    copy_field(sentence, 4U, field, sizeof(field));
    char lon_dir = '\0';
    dir = get_field(sentence, 5U);
    if (dir != NULL) {
        lon_dir = *dir;
    }
    data->longitude = parse_coordinate(field, lon_dir);

    /* Field 6: Fix quality */
    copy_field(sentence, 6U, field, sizeof(field));
    data->fix_quality = (uint8_t)atoi(field);
    data->fix_valid = (data->fix_quality > 0U);

    /* Field 7: Number of satellites */
    copy_field(sentence, 7U, field, sizeof(field));
    data->satellites = (uint8_t)atoi(field);

    /* Field 8: HDOP */
    copy_field(sentence, 8U, field, sizeof(field));
    data->hdop = (float)atof(field);

    /* Field 9: Altitude */
    copy_field(sentence, 9U, field, sizeof(field));
    data->altitude = (float)atof(field);

    data->valid = true;
    return APP_OK;
}

/**
 * @brief Parse RMC sentence (Recommended minimum)
 */
static app_err_t parse_rmc(const char *sentence, nmea_data_t *data)
{
    char field[20];

    data->type = NMEA_RMC;

    /* Field 1: Time */
    copy_field(sentence, 1U, field, sizeof(field));
    parse_time(field, data);

    /* Field 2: Status (A=valid, V=void) */
    copy_field(sentence, 2U, field, sizeof(field));
    data->fix_valid = (field[0] == 'A');

    /* Field 3-4: Latitude */
    copy_field(sentence, 3U, field, sizeof(field));
    char lat_dir = '\0';
    const char *dir = get_field(sentence, 4U);
    if (dir != NULL) {
        lat_dir = *dir;
    }
    data->latitude = parse_coordinate(field, lat_dir);

    /* Field 5-6: Longitude */
    copy_field(sentence, 5U, field, sizeof(field));
    char lon_dir = '\0';
    dir = get_field(sentence, 6U);
    if (dir != NULL) {
        lon_dir = *dir;
    }
    data->longitude = parse_coordinate(field, lon_dir);

    /* Field 7: Speed over ground (knots) */
    copy_field(sentence, 7U, field, sizeof(field));
    data->speed_knots = (float)atof(field);
    data->speed_kmh = data->speed_knots * 1.852f;

    /* Field 8: Course over ground */
    copy_field(sentence, 8U, field, sizeof(field));
    data->course = (float)atof(field);

    /* Field 9: Date */
    copy_field(sentence, 9U, field, sizeof(field));
    parse_date(field, data);

    data->valid = true;
    return APP_OK;
}

/**
 * @brief Parse GSA sentence (DOP and active satellites)
 */
static app_err_t parse_gsa(const char *sentence, nmea_data_t *data)
{
    char field[20];

    data->type = NMEA_GSA;

    /* Field 2: Fix type (1=none, 2=2D, 3=3D) */
    copy_field(sentence, 2U, field, sizeof(field));
    uint8_t fix_type = (uint8_t)atoi(field);
    data->fix_valid = (fix_type >= 2U);

    /* Fields 3-14: Satellite PRNs (skip) */

    /* Field 15: PDOP */
    copy_field(sentence, 15U, field, sizeof(field));
    data->pdop = (float)atof(field);

    /* Field 16: HDOP */
    copy_field(sentence, 16U, field, sizeof(field));
    data->hdop = (float)atof(field);

    /* Field 17: VDOP */
    copy_field(sentence, 17U, field, sizeof(field));
    data->vdop = (float)atof(field);

    data->valid = true;
    return APP_OK;
}

/**
 * @brief Parse VTG sentence (Track and ground speed)
 */
static app_err_t parse_vtg(const char *sentence, nmea_data_t *data)
{
    char field[20];

    data->type = NMEA_VTG;

    /* Field 1: True course */
    copy_field(sentence, 1U, field, sizeof(field));
    data->course = (float)atof(field);

    /* Field 5: Speed in knots */
    copy_field(sentence, 5U, field, sizeof(field));
    data->speed_knots = (float)atof(field);

    /* Field 7: Speed in km/h */
    copy_field(sentence, 7U, field, sizeof(field));
    data->speed_kmh = (float)atof(field);

    data->valid = true;
    return APP_OK;
}

/**
 * @brief Identify sentence type
 */
static nmea_type_t identify_sentence(const char *sentence)
{
    if (strlen(sentence) < 6U) {
        return NMEA_UNKNOWN;
    }

    /* Skip talker ID (GP, GL, GN, etc.) */
    const char *type = &sentence[3];

    if (strncmp(type, "GGA", 3) == 0) {
        return NMEA_GGA;
    }
    if (strncmp(type, "RMC", 3) == 0) {
        return NMEA_RMC;
    }
    if (strncmp(type, "GSA", 3) == 0) {
        return NMEA_GSA;
    }
    if (strncmp(type, "GSV", 3) == 0) {
        return NMEA_GSV;
    }
    if (strncmp(type, "VTG", 3) == 0) {
        return NMEA_VTG;
    }
    if (strncmp(type, "GLL", 3) == 0) {
        return NMEA_GLL;
    }
    if (strncmp(type, "ZDA", 3) == 0) {
        return NMEA_ZDA;
    }

    return NMEA_UNKNOWN;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void nmea_parser_init(void)
{
    state = STATE_IDLE;
    sentence_idx = 0U;
    running_checksum = 0U;
    received_checksum = 0U;
    position_valid = false;
    time_valid = false;
    (void)memset(&last_data, 0, sizeof(last_data));
}

bool nmea_parser_char(char c)
{
    bool sentence_complete = false;

    switch (state) {
    case STATE_IDLE:
        if (c == '$') {
            state = STATE_SENTENCE;
            sentence_idx = 0U;
            running_checksum = 0U;
        }
        break;

    case STATE_SENTENCE:
        if (c == '*') {
            sentence_buf[sentence_idx] = '\0';
            state = STATE_CHECKSUM1;
        } else if ((c == '\r') || (c == '\n')) {
            /* End without checksum */
            sentence_buf[sentence_idx] = '\0';
            state = STATE_IDLE;
        } else if (sentence_idx < (NMEA_MAX_SENTENCE_LEN - 1U)) {
            sentence_buf[sentence_idx] = c;
            sentence_idx++;
            running_checksum ^= (uint8_t)c;
        } else {
            /* Buffer overflow, reset */
            state = STATE_IDLE;
        }
        break;

    case STATE_CHECKSUM1:
        received_checksum = hex_to_val(c) << 4;
        state = STATE_CHECKSUM2;
        break;

    case STATE_CHECKSUM2:
        received_checksum |= hex_to_val(c);

        if (received_checksum == running_checksum) {
            /* Valid checksum, parse sentence */
            nmea_data_t data;
            if (nmea_parser_sentence(sentence_buf, &data) == APP_OK) {
                last_data = data;
                if (data.fix_valid && (data.latitude != 0.0f || data.longitude != 0.0f)) {
                    position_valid = true;
                }
                if (data.hour != 0U || data.minute != 0U) {
                    time_valid = true;
                }
                sentence_complete = true;
            }
        } else {
            LOG_WRN("NMEA checksum mismatch: got 0x%02X, expected 0x%02X",
                    received_checksum, running_checksum);
        }

        state = STATE_IDLE;
        break;

    default:
        state = STATE_IDLE;
        break;
    }

    return sentence_complete;
}

app_err_t nmea_parser_sentence(const char *sentence, nmea_data_t *data)
{
    if ((sentence == NULL) || (data == NULL)) {
        return APP_ERR_INVALID_PARAM;
    }

    (void)memset(data, 0, sizeof(nmea_data_t));

    nmea_type_t type = identify_sentence(sentence);

    switch (type) {
    case NMEA_GGA:
        return parse_gga(sentence, data);
    case NMEA_RMC:
        return parse_rmc(sentence, data);
    case NMEA_GSA:
        return parse_gsa(sentence, data);
    case NMEA_VTG:
        return parse_vtg(sentence, data);
    case NMEA_GSV:
    case NMEA_GLL:
    case NMEA_ZDA:
        /* These sentences are recognized but not fully parsed */
        data->type = type;
        data->valid = true;
        return APP_OK;
    case NMEA_UNKNOWN:
    default:
        return APP_ERR_INVALID_PARAM;
    }
}

app_err_t nmea_parser_get_data(nmea_data_t *data)
{
    if (data == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *data = last_data;
    return APP_OK;
}

bool nmea_parser_has_position(void)
{
    return position_valid;
}

bool nmea_parser_has_time(void)
{
    return time_valid;
}

uint8_t nmea_calculate_checksum(const char *sentence)
{
    uint8_t checksum = 0U;

    if (sentence == NULL) {
        return 0U;
    }

    while (*sentence != '\0') {
        checksum ^= (uint8_t)*sentence;
        sentence++;
    }

    return checksum;
}

bool nmea_verify_checksum(const char *sentence)
{
    if ((sentence == NULL) || (sentence[0] != '$')) {
        return false;
    }

    /* Find the asterisk */
    const char *asterisk = strchr(sentence, '*');
    if ((asterisk == NULL) || (strlen(asterisk) < 3U)) {
        return false;
    }

    /* Calculate checksum of data between $ and * */
    uint8_t calc_checksum = 0U;
    const char *ptr = sentence + 1;  /* Skip $ */
    while (ptr < asterisk) {
        calc_checksum ^= (uint8_t)*ptr;
        ptr++;
    }

    /* Parse received checksum */
    uint8_t recv_checksum = (hex_to_val(asterisk[1]) << 4) | hex_to_val(asterisk[2]);

    return (calc_checksum == recv_checksum);
}
