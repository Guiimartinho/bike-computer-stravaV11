/**
 * @file hal_uart.c
 * @brief UART Hardware Abstraction Layer implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "hal/hal_uart.h"

LOG_MODULE_REGISTER(hal_uart, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Maximum NMEA line length */
#define MAX_LINE_LEN    128U

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

/* UART1 - GPS */
static const struct device *uart_gps_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** UART port context structure */
typedef struct {
    const struct device *dev;
    uint8_t rx_buf[HAL_UART_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    char line_buf[MAX_LINE_LEN];
    uint8_t line_idx;
    hal_uart_rx_callback_t rx_callback;
    hal_uart_line_callback_t line_callback;
    bool enabled;
} uart_ctx_t;

/** UART contexts per port */
static uart_ctx_t uart_ctx[HAL_UART_PORT_COUNT];

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief UART ISR callback
 */
static void uart_isr_callback(const struct device *dev, void *user_data)
{
    uart_ctx_t *ctx = (uart_ctx_t *)user_data;

    if ((ctx == NULL) || (ctx->dev != dev)) {
        return;
    }

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t byte;
            int ret = uart_fifo_read(dev, &byte, 1);

            if (ret == 1) {
                /* Store in circular buffer */
                uint16_t next_head = (ctx->rx_head + 1U) % HAL_UART_RX_BUF_SIZE;

                if (next_head != ctx->rx_tail) {
                    ctx->rx_buf[ctx->rx_head] = byte;
                    ctx->rx_head = next_head;
                }

                /* Build line for NMEA parsing */
                if (byte == '\n') {
                    if (ctx->line_idx > 0U) {
                        ctx->line_buf[ctx->line_idx] = '\0';

                        /* Remove trailing CR if present */
                        if ((ctx->line_idx > 0U) &&
                            (ctx->line_buf[ctx->line_idx - 1U] == '\r')) {
                            ctx->line_buf[ctx->line_idx - 1U] = '\0';
                        }

                        /* Call line callback if registered */
                        if (ctx->line_callback != NULL) {
                            ctx->line_callback(ctx->line_buf);
                        }
                    }
                    ctx->line_idx = 0U;
                } else if (byte == '$') {
                    /* Start of NMEA sentence */
                    ctx->line_idx = 0U;
                    ctx->line_buf[ctx->line_idx] = (char)byte;
                    ctx->line_idx++;
                } else if (ctx->line_idx > 0U) {
                    if (ctx->line_idx < (MAX_LINE_LEN - 1U)) {
                        ctx->line_buf[ctx->line_idx] = (char)byte;
                        ctx->line_idx++;
                    }
                } else {
                    /* Ignore bytes before start marker */
                }
            }
        }
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t hal_uart_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Initialize GPS UART */
    if (!device_is_ready(uart_gps_dev)) {
        LOG_ERR("UART GPS device not ready");
        return APP_ERR_NOT_INIT;
    }

    /* Initialize context */
    (void)memset(&uart_ctx[HAL_UART_GPS], 0, sizeof(uart_ctx_t));
    uart_ctx[HAL_UART_GPS].dev = uart_gps_dev;
    uart_ctx[HAL_UART_GPS].enabled = true;

    /* Configure UART interrupt */
    uart_irq_callback_user_data_set(uart_gps_dev, uart_isr_callback,
                                     &uart_ctx[HAL_UART_GPS]);
    uart_irq_rx_enable(uart_gps_dev);

    is_initialized = true;
    LOG_INF("UART HAL initialized");

    return APP_OK;
}

app_err_t hal_uart_transmit(hal_uart_port_t port, const uint8_t *data, size_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((port >= HAL_UART_PORT_COUNT) || (data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    uart_ctx_t *ctx = &uart_ctx[port];

    if (!ctx->enabled) {
        return APP_ERR_NOT_INIT;
    }

    for (size_t i = 0U; i < len; i++) {
        uart_poll_out(ctx->dev, data[i]);
    }

    return APP_OK;
}

app_err_t hal_uart_transmit_str(hal_uart_port_t port, const char *str)
{
    if (str == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    return hal_uart_transmit(port, (const uint8_t *)str, strlen(str));
}

app_err_t hal_uart_register_rx_callback(hal_uart_port_t port,
                                        hal_uart_rx_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (port >= HAL_UART_PORT_COUNT) {
        return APP_ERR_INVALID_PARAM;
    }

    uart_ctx[port].rx_callback = callback;
    return APP_OK;
}

app_err_t hal_uart_register_line_callback(hal_uart_port_t port,
                                          hal_uart_line_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (port >= HAL_UART_PORT_COUNT) {
        return APP_ERR_INVALID_PARAM;
    }

    uart_ctx[port].line_callback = callback;
    return APP_OK;
}

size_t hal_uart_rx_available(hal_uart_port_t port)
{
    if (!is_initialized || (port >= HAL_UART_PORT_COUNT)) {
        return 0U;
    }

    uart_ctx_t *ctx = &uart_ctx[port];
    uint16_t head = ctx->rx_head;
    uint16_t tail = ctx->rx_tail;

    if (head >= tail) {
        return (size_t)(head - tail);
    }
    return (size_t)(HAL_UART_RX_BUF_SIZE - tail + head);
}

app_err_t hal_uart_read_byte(hal_uart_port_t port, uint8_t *byte)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((port >= HAL_UART_PORT_COUNT) || (byte == NULL)) {
        return APP_ERR_INVALID_PARAM;
    }

    uart_ctx_t *ctx = &uart_ctx[port];

    if (ctx->rx_head == ctx->rx_tail) {
        return APP_ERR_NOT_FOUND;
    }

    *byte = ctx->rx_buf[ctx->rx_tail];
    ctx->rx_tail = (ctx->rx_tail + 1U) % HAL_UART_RX_BUF_SIZE;

    return APP_OK;
}

void hal_uart_flush_rx(hal_uart_port_t port)
{
    if (!is_initialized || (port >= HAL_UART_PORT_COUNT)) {
        return;
    }

    uart_ctx_t *ctx = &uart_ctx[port];
    ctx->rx_head = 0U;
    ctx->rx_tail = 0U;
    ctx->line_idx = 0U;
}

app_err_t hal_uart_set_power(hal_uart_port_t port, bool enable)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (port >= HAL_UART_PORT_COUNT) {
        return APP_ERR_INVALID_PARAM;
    }

    uart_ctx_t *ctx = &uart_ctx[port];

    if (enable) {
        uart_irq_rx_enable(ctx->dev);
        ctx->enabled = true;
    } else {
        uart_irq_rx_disable(ctx->dev);
        ctx->enabled = false;
    }

    return APP_OK;
}
