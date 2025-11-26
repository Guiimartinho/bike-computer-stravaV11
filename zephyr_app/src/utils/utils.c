/**
 * @file utils.c
 * @brief Utility functions for stravaV10
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <math.h>

#include "app_types.h"

/* ==========================================================================
 * String Utilities
 * ========================================================================== */

/**
 * @brief Safe string copy with null termination
 */
void utils_strncpy_safe(char *dst, const char *src, size_t dst_size)
{
    if ((dst == NULL) || (src == NULL) || (dst_size == 0U)) {
        return;
    }

    size_t i;
    for (i = 0U; (i < (dst_size - 1U)) && (src[i] != '\0'); i++) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

/* ==========================================================================
 * Math Utilities
 * ========================================================================== */

/**
 * @brief Fast inverse square root (Quake III algorithm)
 */
float utils_fast_inv_sqrt(float number)
{
    union {
        float f;
        uint32_t i;
    } conv = { .f = number };

    conv.i = 0x5F3759DFU - (conv.i >> 1);
    conv.f *= 1.5f - (number * 0.5f * conv.f * conv.f);

    return conv.f;
}

/**
 * @brief Linear interpolation
 */
float utils_lerp(float a, float b, float t)
{
    return a + (t * (b - a));
}

/**
 * @brief Map value from one range to another
 */
float utils_map(float value, float in_min, float in_max, float out_min, float out_max)
{
    float in_range = in_max - in_min;
    if (fabsf(in_range) < 0.0001f) {
        return out_min;
    }

    float t = (value - in_min) / in_range;
    return utils_lerp(out_min, out_max, t);
}

/* ==========================================================================
 * Time Utilities
 * ========================================================================== */

/**
 * @brief Format seconds as HH:MM:SS
 */
void utils_format_time(uint32_t seconds, char *buf, size_t buf_size)
{
    if ((buf == NULL) || (buf_size < 9U)) {
        return;
    }

    uint8_t h = (uint8_t)(seconds / 3600U);
    uint8_t m = (uint8_t)((seconds % 3600U) / 60U);
    uint8_t s = (uint8_t)(seconds % 60U);

    (void)snprintf(buf, buf_size, "%02u:%02u:%02u", h, m, s);
}

/**
 * @brief Format distance in km
 */
void utils_format_distance(float meters, char *buf, size_t buf_size)
{
    if ((buf == NULL) || (buf_size < 10U)) {
        return;
    }

    if (meters < 1000.0f) {
        (void)snprintf(buf, buf_size, "%.0f m", (double)meters);
    } else {
        (void)snprintf(buf, buf_size, "%.2f km", (double)(meters / 1000.0f));
    }
}
