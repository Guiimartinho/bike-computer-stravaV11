/**
 * @file glasses.h
 * @brief ANT+ Smart Glasses Stub Interface
 *
 * IMPORTANT: ANT+ Smart Glasses are NOT available in Zephyr/NCS.
 *
 * The original implementation used ANT+ to transmit data to
 * cycling smart glasses (e.g., custom ANT+ enabled HUDs).
 * This requires the proprietary ANT+ stack which is not in Zephyr.
 *
 * ALTERNATIVES:
 * - Use BLE-based smart glasses if available
 * - Use BLE Advertising for simple data broadcast
 * - Display data directly on the bike computer LCD
 *
 * This header provides stub functions for code compatibility only.
 */

#ifndef RF_GLASSES_H
#define RF_GLASSES_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Glasses display orders (what to show on the glasses HUD) */
typedef struct {
    uint8_t led;            /**< LED indicator pattern */
    uint8_t av_ent;         /**< "Avant Entree" - before segment entry */
    uint8_t av_dec;         /**< "Avant Decompte" - countdown before */
    uint8_t direction;      /**< Navigation direction indicator */
    int8_t time_diff;       /**< Time difference vs target (seconds) */
    uint8_t power_zone;     /**< Current power zone (1-7) */
    bool segment_active;    /**< True if segment is active */
} glasses_orders_t;

/* ==========================================================================
 * Public Functions (Stubs)
 * ========================================================================== */

/**
 * @brief Initialize glasses module (STUB - does nothing)
 * @return APP_OK always
 * @note ANT+ glasses not available in Zephyr
 */
static inline app_err_t glasses_init(void)
{
    /* ANT+ glasses not available in Zephyr */
    return APP_OK;
}

/**
 * @brief Set glasses display buffer (STUB - does nothing)
 * @param orders Display orders to send
 * @note Data is ignored - ANT+ not available
 */
static inline void glasses_set_buffer(const glasses_orders_t *orders)
{
    (void)orders;
    /* ANT+ glasses not available */
}

/**
 * @brief Setup glasses ANT profile (STUB - does nothing)
 */
static inline void glasses_profile_setup(void)
{
    /* Nothing to do */
}

/**
 * @brief Start glasses ANT profile (STUB - does nothing)
 */
static inline void glasses_profile_start(void)
{
    /* Nothing to do */
}

/**
 * @brief Stop glasses ANT profile (STUB - does nothing)
 */
static inline void glasses_profile_stop(void)
{
    /* Nothing to do */
}

/**
 * @brief Check if glasses are connected (STUB - always false)
 * @return false always
 */
static inline bool glasses_is_connected(void)
{
    return false;
}

/**
 * @brief Send segment entry notification to glasses (STUB)
 * @param segment_name Name of segment
 */
static inline void glasses_notify_segment_entry(const char *segment_name)
{
    (void)segment_name;
}

/**
 * @brief Send segment exit notification to glasses (STUB)
 * @param time_diff Time difference vs target
 */
static inline void glasses_notify_segment_exit(int8_t time_diff)
{
    (void)time_diff;
}

/**
 * @brief Update glasses with navigation direction (STUB)
 * @param direction Direction code (0=straight, 1=left, 2=right, etc.)
 */
static inline void glasses_update_direction(uint8_t direction)
{
    (void)direction;
}

#ifdef __cplusplus
}
#endif

#endif /* RF_GLASSES_H */
