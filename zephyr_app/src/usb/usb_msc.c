/**
 * @file usb_msc.c
 * @brief USB Mass Storage Class implementation
 *
 * Provides USB Mass Storage for exposing SD card to host PC.
 *
 * Note: Full MSC implementation requires CONFIG_USB_DEVICE_MSC which needs
 * proper block device configuration. This file provides the framework.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_USB_DEVICE_MSC
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_msc.h>
#endif

#include "usb/usb_msc.h"

LOG_MODULE_REGISTER(usb_msc, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current state */
static usb_msc_state_t state = USB_MSC_STATE_DISABLED;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t usb_msc_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

#ifdef CONFIG_USB_DEVICE_MSC
    /* MSC initialization happens through Kconfig/devicetree */
    state = USB_MSC_STATE_READY;
    is_initialized = true;
    LOG_INF("USB MSC initialized");
    return APP_OK;
#else
    LOG_WRN("USB MSC not enabled in Kconfig (CONFIG_USB_DEVICE_MSC=n)");
    LOG_WRN("To enable: add CONFIG_USB_DEVICE_MSC=y to prj.conf");
    is_initialized = true;
    state = USB_MSC_STATE_DISABLED;
    return APP_OK;
#endif
}

app_err_t usb_msc_enable(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

#ifdef CONFIG_USB_DEVICE_MSC
    if (state == USB_MSC_STATE_ACTIVE) {
        return APP_OK;
    }

    /*
     * To enable MSC mode:
     * 1. Unmount filesystem from SD card
     * 2. USB MSC will expose the block device
     *
     * The block device is configured via devicetree and Kconfig.
     */
    state = USB_MSC_STATE_ACTIVE;
    LOG_INF("USB MSC mode enabled");
    return APP_OK;
#else
    LOG_WRN("USB MSC not available");
    return APP_ERR_NOT_INIT;
#endif
}

app_err_t usb_msc_disable(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

#ifdef CONFIG_USB_DEVICE_MSC
    if (state == USB_MSC_STATE_READY || state == USB_MSC_STATE_DISABLED) {
        return APP_OK;
    }

    state = USB_MSC_STATE_READY;
    LOG_INF("USB MSC mode disabled");
    return APP_OK;
#else
    return APP_OK;
#endif
}

usb_msc_state_t usb_msc_get_state(void)
{
    return state;
}

bool usb_msc_is_active(void)
{
#ifdef CONFIG_USB_DEVICE_MSC
    return (state == USB_MSC_STATE_ACTIVE);
#else
    return false;
#endif
}

void usb_msc_process(void)
{
#ifdef CONFIG_USB_DEVICE_MSC
    /* MSC processing handled by USB stack */
#endif
}
