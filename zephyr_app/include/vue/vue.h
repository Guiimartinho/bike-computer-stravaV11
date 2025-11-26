/**
 * @file vue.h
 * @brief Display/View management for stravaV10
 *
 * Manages screen rendering and page navigation.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef VUE_VUE_H
#define VUE_VUE_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Screen page enumeration */
typedef enum {
    VUE_PAGE_MAIN = 0,      /**< Main cycling data */
    VUE_PAGE_SEGMENT,       /**< Segment info */
    VUE_PAGE_MAP,           /**< Map view */
    VUE_PAGE_STATS,         /**< Statistics */
    VUE_PAGE_SENSORS,       /**< Sensor status */
    VUE_PAGE_MENU,          /**< Settings menu */
    VUE_PAGE_COUNT
} vue_page_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize display/view system
 * @return APP_OK on success, error code otherwise
 */
app_err_t vue_init(void);

/**
 * @brief Update display with current data
 */
void vue_update(void);

/**
 * @brief Force full screen redraw
 */
void vue_refresh(void);

/**
 * @brief Set current view mode
 * @param mode View mode
 */
void vue_set_mode(vue_mode_t mode);

/**
 * @brief Get current view mode
 * @return Current mode
 */
vue_mode_t vue_get_mode(void);

/**
 * @brief Switch to next page
 */
void vue_next_page(void);

/**
 * @brief Switch to previous page
 */
void vue_prev_page(void);

/**
 * @brief Set current page
 * @param page Page to display
 */
void vue_set_page(vue_page_t page);

/**
 * @brief Get current page
 * @return Current page
 */
vue_page_t vue_get_page(void);

/**
 * @brief Handle button event
 * @param event Button event
 */
void vue_handle_button(btn_event_t event);

/**
 * @brief Show notification
 * @param notif Notification data
 */
void vue_show_notification(const notification_t *notif);

/**
 * @brief Clear notification
 */
void vue_clear_notification(void);

/**
 * @brief Toggle backlight (if available)
 */
void vue_toggle_backlight(void);

/**
 * @brief Set display brightness
 * @param level Brightness level (0-100)
 */
void vue_set_brightness(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* VUE_VUE_H */
