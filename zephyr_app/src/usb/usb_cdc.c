/**
 * @file usb_cdc.c
 * @brief USB CDC ACM (Serial over USB) implementation
 *
 * Provides serial communication over USB using Zephyr's USB device stack.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usb/usb_cdc.h"

LOG_MODULE_REGISTER(usb_cdc, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** CDC ACM device name */
#define CDC_ACM_DEV_NAME    "cdc_acm_uart0"

/** Printf buffer size */
#define PRINTF_BUF_SIZE     256U

/** TX trigger threshold (send when this many bytes buffered) */
#define TX_TRIGGER_SIZE     64U

/** TX timeout in ms */
#define TX_TIMEOUT_MS       10U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** CDC ACM device */
static const struct device *cdc_dev;

/** Current state */
static usb_cdc_state_t state = USB_CDC_STATE_DISABLED;

/** Initialization flag */
static bool is_initialized;

/** DTR (Data Terminal Ready) signal */
static bool dtr_set;

/** RX callback */
static usb_cdc_rx_callback_t rx_callback;

/** TX ring buffer */
static uint8_t tx_buffer[USB_CDC_TX_BUF_SIZE];
static uint16_t tx_head;
static uint16_t tx_tail;

/** RX buffer */
static uint8_t rx_buffer[USB_CDC_RX_BUF_SIZE];

/** Printf scratch buffer */
static char printf_buf[PRINTF_BUF_SIZE];

/** Mutex for TX buffer */
static K_MUTEX_DEFINE(tx_mutex);

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Get number of bytes in TX buffer
 */
static uint16_t tx_buffer_count(void)
{
    if (tx_head >= tx_tail) {
        return tx_head - tx_tail;
    }
    return USB_CDC_TX_BUF_SIZE - tx_tail + tx_head;
}

/**
 * @brief Add byte to TX buffer
 */
static bool tx_buffer_put(uint8_t byte)
{
    uint16_t next = (tx_head + 1U) % USB_CDC_TX_BUF_SIZE;

    if (next == tx_tail) {
        return false; /* Buffer full */
    }

    tx_buffer[tx_head] = byte;
    tx_head = next;
    return true;
}

/**
 * @brief Get byte from TX buffer
 */
static bool tx_buffer_get(uint8_t *byte)
{
    if (tx_head == tx_tail) {
        return false; /* Buffer empty */
    }

    *byte = tx_buffer[tx_tail];
    tx_tail = (tx_tail + 1U) % USB_CDC_TX_BUF_SIZE;
    return true;
}

/**
 * @brief UART interrupt callback
 */
static void uart_irq_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        /* Handle RX */
        if (uart_irq_rx_ready(dev)) {
            int recv_len;

            recv_len = uart_fifo_read(dev, rx_buffer, sizeof(rx_buffer));
            if ((recv_len > 0) && (rx_callback != NULL)) {
                rx_callback(rx_buffer, (uint16_t)recv_len);
            }
        }

        /* Handle TX */
        if (uart_irq_tx_ready(dev)) {
            uint8_t byte;

            k_mutex_lock(&tx_mutex, K_FOREVER);
            if (tx_buffer_get(&byte)) {
                uart_fifo_fill(dev, &byte, 1);
            } else {
                uart_irq_tx_disable(dev);
            }
            k_mutex_unlock(&tx_mutex);
        }
    }
}

/**
 * @brief USB status callback
 */
static void usb_status_callback(enum usb_dc_status_code status,
                                const uint8_t *param)
{
    ARG_UNUSED(param);

    switch (status) {
    case USB_DC_ERROR:
        LOG_ERR("USB error");
        state = USB_CDC_STATE_DISCONNECTED;
        break;

    case USB_DC_RESET:
        LOG_INF("USB reset");
        state = USB_CDC_STATE_DISCONNECTED;
        dtr_set = false;
        break;

    case USB_DC_CONNECTED:
        LOG_INF("USB connected");
        state = USB_CDC_STATE_CONNECTED;
        break;

    case USB_DC_CONFIGURED:
        LOG_INF("USB configured");
        state = USB_CDC_STATE_CONFIGURED;
        break;

    case USB_DC_DISCONNECTED:
        LOG_INF("USB disconnected");
        state = USB_CDC_STATE_DISCONNECTED;
        dtr_set = false;
        break;

    case USB_DC_SUSPEND:
        LOG_DBG("USB suspend");
        break;

    case USB_DC_RESUME:
        LOG_DBG("USB resume");
        break;

    default:
        LOG_DBG("USB status: %d", status);
        break;
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t usb_cdc_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Get CDC ACM device */
    cdc_dev = device_get_binding(CDC_ACM_DEV_NAME);
    if (cdc_dev == NULL) {
        /* Try alternative name */
        cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
        if (!device_is_ready(cdc_dev)) {
            LOG_ERR("CDC ACM device not found");
            return APP_ERR_NOT_FOUND;
        }
    }

    /* Initialize buffers */
    tx_head = 0U;
    tx_tail = 0U;
    dtr_set = false;

    is_initialized = true;
    state = USB_CDC_STATE_DISABLED;

    LOG_INF("USB CDC initialized");
    return APP_OK;
}

app_err_t usb_cdc_enable(void)
{
    int ret;

    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (state != USB_CDC_STATE_DISABLED) {
        return APP_OK; /* Already enabled */
    }

    /* Enable USB subsystem */
    ret = usb_enable(usb_status_callback);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return APP_ERR_IO;
    }

    /* Set up UART interrupt callback */
    uart_irq_callback_set(cdc_dev, uart_irq_callback);
    uart_irq_rx_enable(cdc_dev);

    state = USB_CDC_STATE_DISCONNECTED;
    LOG_INF("USB CDC enabled");

    return APP_OK;
}

app_err_t usb_cdc_disable(void)
{
    int ret;

    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (state == USB_CDC_STATE_DISABLED) {
        return APP_OK;
    }

    uart_irq_rx_disable(cdc_dev);
    uart_irq_tx_disable(cdc_dev);

    ret = usb_disable();
    if (ret != 0) {
        LOG_WRN("Failed to disable USB: %d", ret);
    }

    state = USB_CDC_STATE_DISABLED;
    LOG_INF("USB CDC disabled");

    return APP_OK;
}

usb_cdc_state_t usb_cdc_get_state(void)
{
    return state;
}

bool usb_cdc_is_ready(void)
{
    uint32_t dtr_val = 0U;

    if (state != USB_CDC_STATE_CONFIGURED) {
        return false;
    }

    /* Check DTR signal */
    (void)uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr_val);
    dtr_set = (dtr_val != 0U);

    return dtr_set;
}

void usb_cdc_register_rx_callback(usb_cdc_rx_callback_t callback)
{
    rx_callback = callback;
}

app_err_t usb_cdc_send(const uint8_t *data, uint16_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    k_mutex_lock(&tx_mutex, K_FOREVER);

    for (uint16_t i = 0U; i < len; i++) {
        if (!tx_buffer_put(data[i])) {
            k_mutex_unlock(&tx_mutex);
            return APP_ERR_OVERFLOW;
        }
    }

    /* Enable TX interrupt if we have data and CDC is ready */
    if (usb_cdc_is_ready() && (tx_buffer_count() > 0U)) {
        uart_irq_tx_enable(cdc_dev);
    }

    k_mutex_unlock(&tx_mutex);
    return APP_OK;
}

app_err_t usb_cdc_send_str(const char *str)
{
    if (str == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    return usb_cdc_send((const uint8_t *)str, (uint16_t)strlen(str));
}

int usb_cdc_printf(const char *format, ...)
{
    va_list args;
    int len;

    if (format == NULL) {
        return -1;
    }

    va_start(args, format);
    len = vsnprintf(printf_buf, sizeof(printf_buf), format, args);
    va_end(args);

    if (len > 0) {
        if (len >= (int)sizeof(printf_buf)) {
            len = (int)sizeof(printf_buf) - 1;
        }

        app_err_t err = usb_cdc_send((const uint8_t *)printf_buf, (uint16_t)len);
        if (err != APP_OK) {
            return -1;
        }
    }

    return len;
}

void usb_cdc_putc(char c)
{
    k_mutex_lock(&tx_mutex, K_FOREVER);
    (void)tx_buffer_put((uint8_t)c);

    if (usb_cdc_is_ready()) {
        uart_irq_tx_enable(cdc_dev);
    }
    k_mutex_unlock(&tx_mutex);
}

void usb_cdc_flush(void)
{
    if (!is_initialized || !usb_cdc_is_ready()) {
        return;
    }

    k_mutex_lock(&tx_mutex, K_FOREVER);

    /* Trigger TX if data pending */
    if (tx_buffer_count() > 0U) {
        uart_irq_tx_enable(cdc_dev);
    }

    k_mutex_unlock(&tx_mutex);

    /* Wait for buffer to drain */
    uint32_t timeout = 100U; /* ms */
    while ((tx_buffer_count() > 0U) && (timeout > 0U)) {
        k_msleep(1);
        timeout--;
    }
}

void usb_cdc_process(void)
{
    if (!is_initialized) {
        return;
    }

    /* Check for DTR changes */
    bool was_ready = dtr_set;
    bool is_ready = usb_cdc_is_ready();

    if (is_ready && !was_ready) {
        LOG_INF("USB CDC port opened");
        /* Could send welcome message here */
    } else if (!is_ready && was_ready) {
        LOG_INF("USB CDC port closed");
    }

    /* If we have pending data and CDC is ready, trigger TX */
    if (is_ready) {
        k_mutex_lock(&tx_mutex, K_FOREVER);
        if (tx_buffer_count() > 0U) {
            uart_irq_tx_enable(cdc_dev);
        }
        k_mutex_unlock(&tx_mutex);
    }
}
