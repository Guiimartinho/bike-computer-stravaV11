/**
 * @file usb_cdc.h
 * @brief USB CDC ACM (Serial over USB) interface
 *
 * Provides serial communication over USB for debugging and data transfer.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** TX ring buffer size */
#define USB_CDC_TX_BUF_SIZE     1024U

/** RX ring buffer size */
#define USB_CDC_RX_BUF_SIZE     256U

/** USB CDC work queue stack size */
#define USB_CDC_STACK_SIZE      1024U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** USB CDC state */
typedef enum {
    USB_CDC_STATE_DISABLED = 0,
    USB_CDC_STATE_DISCONNECTED,
    USB_CDC_STATE_CONNECTED,
    USB_CDC_STATE_CONFIGURED
} usb_cdc_state_t;

/** USB CDC RX callback type */
typedef void (*usb_cdc_rx_callback_t)(const uint8_t *data, uint16_t len);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize USB CDC subsystem
 * @return APP_OK on success
 */
app_err_t usb_cdc_init(void);

/**
 * @brief Enable USB CDC (starts enumeration)
 * @return APP_OK on success
 */
app_err_t usb_cdc_enable(void);

/**
 * @brief Disable USB CDC
 * @return APP_OK on success
 */
app_err_t usb_cdc_disable(void);

/**
 * @brief Get current USB CDC state
 * @return Current state
 */
usb_cdc_state_t usb_cdc_get_state(void);

/**
 * @brief Check if USB CDC is connected and ready
 * @return true if ready to send/receive
 */
bool usb_cdc_is_ready(void);

/**
 * @brief Register RX callback
 * @param callback Function to call when data received
 */
void usb_cdc_register_rx_callback(usb_cdc_rx_callback_t callback);

/**
 * @brief Send data over USB CDC
 * @param data Data to send
 * @param len Length of data
 * @return APP_OK on success, APP_ERR_BUSY if buffer full
 */
app_err_t usb_cdc_send(const uint8_t *data, uint16_t len);

/**
 * @brief Send string over USB CDC
 * @param str Null-terminated string
 * @return APP_OK on success
 */
app_err_t usb_cdc_send_str(const char *str);

/**
 * @brief Print formatted string to USB CDC
 * @param format Printf format string
 * @param ... Arguments
 * @return Number of characters written, or negative on error
 */
int usb_cdc_printf(const char *format, ...);

/**
 * @brief Print single character
 * @param c Character to print
 */
void usb_cdc_putc(char c);

/**
 * @brief Flush TX buffer (send pending data)
 */
void usb_cdc_flush(void);

/**
 * @brief Process USB CDC events (call periodically or from work queue)
 */
void usb_cdc_process(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_H */
