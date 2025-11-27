/**
 * @file usb_msc.h
 * @brief USB Mass Storage Class interface
 *
 * Provides USB Mass Storage for exposing SD card over USB.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef USB_MSC_H
#define USB_MSC_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** USB MSC state */
typedef enum {
    USB_MSC_STATE_DISABLED = 0,
    USB_MSC_STATE_READY,
    USB_MSC_STATE_CONNECTED,
    USB_MSC_STATE_ACTIVE
} usb_msc_state_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize USB MSC subsystem
 * @return APP_OK on success
 */
app_err_t usb_msc_init(void);

/**
 * @brief Enable USB MSC mode
 * @note This will unmount the SD card from the filesystem
 * @return APP_OK on success
 */
app_err_t usb_msc_enable(void);

/**
 * @brief Disable USB MSC mode
 * @note This allows the filesystem to remount the SD card
 * @return APP_OK on success
 */
app_err_t usb_msc_disable(void);

/**
 * @brief Get current USB MSC state
 * @return Current state
 */
usb_msc_state_t usb_msc_get_state(void);

/**
 * @brief Check if USB MSC is active
 * @return true if MSC is active and connected
 */
bool usb_msc_is_active(void);

/**
 * @brief Process USB MSC events
 */
void usb_msc_process(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_MSC_H */
