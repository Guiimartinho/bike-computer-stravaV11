/**
 * @file rr_zone.c
 * @brief RR interval (HRV) zone tracking implementation
 *
 * Based on original RRZone.cpp implementation.
 * Calculates RMSSD (Root Mean Square of Successive Differences)
 * for Heart Rate Variability analysis by HR zone.
 */

#include <string.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "model/rr_zone.h"

LOG_MODULE_REGISTER(rr_zone, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** HR zone limits for RR binning (from original) */
static const float rr_lims[RR_ZONES_NB + 1U] = {
    -1000.0f,   /* Below Z1 */
    70.0f,      /* Z1 upper: 70 bpm */
    108.0f,     /* Z2 upper: 108 bpm */
    143.0f,     /* Z3 upper: 143 bpm */
    161.0f,     /* Z4 upper: 161 bpm */
    178.0f,     /* Z5 upper: 178 bpm */
    1000.0f     /* Z6 upper: unlimited */
};

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate RMSSD from RR buffer
 * @param rz Pointer to RR zone structure
 * @return RMSSD value
 */
static float calculate_rmssd(const rr_zone_t *rz)
{
    if (rz->buffer_count < 2U) {
        return 0.0f;
    }

    float sum_sq = 0.0f;

    for (uint16_t i = 1U; i < rz->buffer_count; i++) {
        float diff = rz->rr_buffer[i] - rz->rr_buffer[i - 1U];
        sum_sq += diff * diff;
    }

    /* RMSSD = sqrt(mean of squared differences) */
    float rmssd = sqrtf(sum_sq / (float)rz->buffer_count);

    return rmssd;
}

/**
 * @brief Find HR zone for given BPM
 * @param bpm Heart rate in BPM
 * @return Zone index (0 to RR_ZONES_NB-1), or RR_ZONES_NB if not found
 */
static uint8_t find_hr_zone(uint8_t bpm)
{
    float bpm_f = (float)bpm;

    for (uint8_t i = 0U; i < RR_ZONES_NB; i++) {
        if ((bpm_f >= rr_lims[i]) && (bpm_f < rr_lims[i + 1U])) {
            return i;
        }
    }

    return RR_ZONES_NB; /* Invalid zone */
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void rr_zone_init(rr_zone_t *rz)
{
    if (rz == NULL) {
        return;
    }

    (void)memset(rz, 0, sizeof(rr_zone_t));

    LOG_INF("RR zone initialized");
}

void rr_zone_reset(rr_zone_t *rz)
{
    if (rz == NULL) {
        return;
    }

    (void)memset(rz, 0, sizeof(rr_zone_t));
}

void rr_zone_add_data(rr_zone_t *rz, const hrm_info_t *hrm_info)
{
    if ((rz == NULL) || (hrm_info == NULL)) {
        return;
    }

    /* Invalid timestamp */
    if (hrm_info->timestamp == 0U) {
        return;
    }

    /* First call - initialize timestamp */
    if (rz->last_timestamp == 0U) {
        rz->last_timestamp = hrm_info->timestamp;
        return;
    }

    /* Save RR data to buffer */
    rz->rr_buffer[rz->buffer_count] = (float)hrm_info->rr_interval;
    rz->buffer_count++;

    /* Check if buffer is full */
    if (rz->buffer_count < RR_VAR_NB_ELEM) {
        return;
    }

    /* Buffer is full - calculate RMSSD */
    float rmssd = calculate_rmssd(rz);

    LOG_DBG("Measured RR RMSSD: %d", (int)rmssd);

    /* Reset buffer */
    rz->buffer_count = 0U;

    /* Update timestamp */
    rz->last_timestamp = hrm_info->timestamp;

    /* Find the HR zone and accumulate RMSSD */
    uint8_t zone = find_hr_zone(hrm_info->bpm);

    if (zone < RR_ZONES_NB) {
        rz->rr_bins[zone] += rmssd;
        rz->tm_bins[zone] += 1.0f;
        rz->last_bin = zone;

        LOG_DBG("Logging RR in bin %u (RMSSD=%.1f)", zone, (double)rmssd);
    } else {
        LOG_WRN("Wrong bin RR: bpm=%u rmssd=%d", hrm_info->bpm, (int)rmssd);
    }
}

float rr_zone_get_value(const rr_zone_t *rz, uint8_t zone)
{
    if ((rz == NULL) || (zone >= RR_ZONES_NB)) {
        return 0.0f;
    }

    /* Return average RMSSD for zone */
    if (rz->tm_bins[zone] > 0.0f) {
        return rz->rr_bins[zone] / rz->tm_bins[zone];
    }

    return 0.0f;
}

uint32_t rr_zone_get_time(const rr_zone_t *rz, uint8_t zone)
{
    if ((rz == NULL) || (zone >= RR_ZONES_NB)) {
        return 0U;
    }

    return (uint32_t)rz->tm_bins[zone];
}

uint32_t rr_zone_get_total_time(const rr_zone_t *rz)
{
    if (rz == NULL) {
        return 0U;
    }

    float total = 0.0f;

    for (uint8_t i = 0U; i < RR_ZONES_NB; i++) {
        total += rz->tm_bins[i];
    }

    return (uint32_t)total;
}

float rr_zone_get_max_value(const rr_zone_t *rz)
{
    if (rz == NULL) {
        return 0.0f;
    }

    float max_val = rr_zone_get_value(rz, 0U);

    for (uint8_t i = 1U; i < RR_ZONES_NB; i++) {
        float val = rr_zone_get_value(rz, i);
        if (val > max_val) {
            max_val = val;
        }
    }

    return max_val;
}

uint32_t rr_zone_get_max_time(const rr_zone_t *rz)
{
    if (rz == NULL) {
        return 0U;
    }

    float max_time = rz->tm_bins[0];

    for (uint8_t i = 1U; i < RR_ZONES_NB; i++) {
        if (rz->tm_bins[i] > max_time) {
            max_time = rz->tm_bins[i];
        }
    }

    return (uint32_t)max_time;
}

uint8_t rr_zone_get_current(const rr_zone_t *rz)
{
    if (rz == NULL) {
        return 0U;
    }

    return rz->last_bin;
}

uint8_t rr_zone_get_count(void)
{
    return RR_ZONES_NB;
}
