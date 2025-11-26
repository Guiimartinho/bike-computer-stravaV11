/**
 * @file power_zone.c
 * @brief Power zone tracking implementation
 *
 * Based on original PowerZone.cpp implementation.
 */

#include <string.h>
#include <zephyr/logging/log.h>

#include "model/power_zone.h"

LOG_MODULE_REGISTER(power_zone, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Power zone limits as percentage of FTP (from original) */
static const float pw_limits[PW_ZONES_NB + 1U] = {
    -100.0f,    /* Below Z1 */
    0.55f,      /* Z1 upper: 55% */
    0.75f,      /* Z2 upper: 75% */
    0.90f,      /* Z3 upper: 90% */
    1.05f,      /* Z4 upper: 105% */
    1.20f,      /* Z5 upper: 120% */
    1.50f,      /* Z6 upper: 150% */
    100.0f      /* Z7 upper: unlimited */
};

/** Minimum valid power */
#define MIN_POWER_WATTS     50U

/** Maximum valid power */
#define MAX_POWER_WATTS     1950U

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void power_zone_init(power_zone_t *pz, uint16_t ftp)
{
    if (pz == NULL) {
        return;
    }

    (void)memset(pz, 0, sizeof(power_zone_t));
    pz->ftp = ftp;

    LOG_INF("Power zone initialized (FTP=%u)", ftp);
}

void power_zone_reset(power_zone_t *pz)
{
    if (pz == NULL) {
        return;
    }

    uint16_t ftp = pz->ftp;
    (void)memset(pz, 0, sizeof(power_zone_t));
    pz->ftp = ftp;
}

void power_zone_add_data(power_zone_t *pz, uint16_t power_watts, uint32_t timestamp_ms)
{
    if (pz == NULL) {
        return;
    }

    /* First call - initialize timestamp */
    if (pz->last_timestamp == 0U) {
        pz->last_timestamp = timestamp_ms;
        return;
    }

    /* Validate power range */
    if ((power_watts < MIN_POWER_WATTS) || (power_watts > MAX_POWER_WATTS)) {
        pz->last_timestamp = timestamp_ms;
        return;
    }

    /* Calculate elapsed time */
    float time_delta = (float)(timestamp_ms - pz->last_timestamp) / 1000.0f;

    if (time_delta <= 0.0f) {
        return;
    }

    /* Find the zone */
    float ftp_f = (float)pz->ftp;

    for (uint8_t i = 0U; i < PW_ZONES_NB; i++) {
        float lower = pw_limits[i] * ftp_f;
        float upper = pw_limits[i + 1U] * ftp_f;

        if (((float)power_watts >= lower) && ((float)power_watts < upper)) {
            pz->pw_bins[i] += time_delta;
            pz->last_bin = i;

            LOG_DBG("Power %uW -> Zone %u (%.1fs)", power_watts, i + 1U, (double)time_delta);

            pz->last_timestamp = timestamp_ms;
            return;
        }
    }

    /* Should not reach here */
    LOG_WRN("Power %uW outside all zones (FTP=%u)", power_watts, pz->ftp);
    pz->last_timestamp = timestamp_ms;
}

uint32_t power_zone_get_time(const power_zone_t *pz, uint8_t zone)
{
    if ((pz == NULL) || (zone >= PW_ZONES_NB)) {
        return 0U;
    }

    return (uint32_t)pz->pw_bins[zone];
}

uint32_t power_zone_get_total_time(const power_zone_t *pz)
{
    if (pz == NULL) {
        return 0U;
    }

    float total = 0.0f;

    for (uint8_t i = 0U; i < PW_ZONES_NB; i++) {
        total += pz->pw_bins[i];
    }

    return (uint32_t)total;
}

uint32_t power_zone_get_max_time(const power_zone_t *pz)
{
    if (pz == NULL) {
        return 0U;
    }

    float max_time = pz->pw_bins[0];

    for (uint8_t i = 1U; i < PW_ZONES_NB; i++) {
        if (pz->pw_bins[i] > max_time) {
            max_time = pz->pw_bins[i];
        }
    }

    return (uint32_t)max_time;
}

uint8_t power_zone_get_current(const power_zone_t *pz)
{
    if (pz == NULL) {
        return 0U;
    }

    return pz->last_bin;
}

uint8_t power_zone_get_count(void)
{
    return PW_ZONES_NB;
}

void power_zone_set_ftp(power_zone_t *pz, uint16_t ftp)
{
    if (pz == NULL) {
        return;
    }

    pz->ftp = ftp;
    LOG_INF("FTP updated to %u", ftp);
}
