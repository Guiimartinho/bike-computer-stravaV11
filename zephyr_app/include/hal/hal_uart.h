/**
 * @file hal_uart.h
 * @brief UART Hardware Abstraction Layer for stravaV10
 *
 * Provides abstracted UART operations for GPS module.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** UART RX buffer size */
#define HAL_UART_RX_BUF_SIZE    256U

/** UART TX buffer size */
#define HAL_UART_TX_BUF_SIZE    128U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** UART port identifiers */
typedef enum {
    HAL_UART_GPS = 0,       /**< UART for GPS module (UART1) */
    HAL_UART_PORT_COUNT
} hal_uart_port_t;

/** UART receive callback function type */
typedef void (*hal_uart_rx_callback_t)(const uint8_t *data, size_t len);

/** UART line received callback function type */
typedef void (*hal_uart_line_callback_t)(const char *line);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize UART subsystem
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_uart_init(void);

/**
 * @brief Transmit data over UART
 * @param port UART port identifier
 * @param data Data buffer to transmit
 * @param len Number of bytes to transmit
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_uart_transmit(hal_uart_port_t port, const uint8_t *data, size_t len);

/**
 * @brief Transmit string over UART
 * @param port UART port identifier
 * @param str Null-terminated string to transmit
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_uart_transmit_str(hal_uart_port_t port, const char *str);

/**
 * @brief Register callback for received data
 * @param port UART port identifier
 * @param callback Function to call when data is received
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_uart_register_rx_callback(hal_uart_port_t port,
                                        hal_uart_rx_callback_t callback);

/**
 * @brief Register callback for received lines (NMEA sentences)
 * @param port UART port identifier
 * @param callback Function to call when a complete line is received
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_uart_register_line_callback(hal_uart_port_t port,
                                          hal_uart_line_callback_t callback);

/**
 * @brief Get number of bytes available in RX buffer
 * @param port UART port identifier
 * @return Number of bytes available
 */
size_t hal_uart_rx_available(hal_uart_port_t port);

/**
 * @brief Read byte from RX buffer
 * @param port UART port identifier
 * @param byte Pointer to store read byte
 * @return APP_OK on success, APP_ERR_NOT_FOUND if buffer empty
 */
app_err_t hal_uart_read_byte(hal_uart_port_t port, uint8_t *byte);

/**
 * @brief Flush UART RX buffer
 * @param port UART port identifier
 */
void hal_uart_flush_rx(hal_uart_port_t port);

/**
 * @brief Enable/disable UART power
 * @param port UART port identifier
 * @param enable true to enable, false to disable
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_uart_set_power(hal_uart_port_t port, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
