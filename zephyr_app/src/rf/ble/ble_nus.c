/**
 * @file ble_nus.c
 * @brief Nordic UART Service implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "rf/ble_nus.h"

LOG_MODULE_REGISTER(ble_nus, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** NUS Service UUID */
#define BT_UUID_NUS_VAL \
    BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

/** NUS TX Characteristic UUID */
#define BT_UUID_NUS_TX_VAL \
    BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

/** NUS RX Characteristic UUID */
#define BT_UUID_NUS_RX_VAL \
    BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

#define BT_UUID_NUS_SERVICE   BT_UUID_DECLARE_128(BT_UUID_NUS_VAL)
#define BT_UUID_NUS_TX        BT_UUID_DECLARE_128(BT_UUID_NUS_TX_VAL)
#define BT_UUID_NUS_RX        BT_UUID_DECLARE_128(BT_UUID_NUS_RX_VAL)

/** Maximum NUS packet size */
#define NUS_MAX_LEN           244U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** TX notifications enabled */
static volatile bool tx_notifications_enabled;

/** Receive callback */
static ble_nus_rx_callback_t rx_callback;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * GATT Callbacks
 * ========================================================================== */

static void nus_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    tx_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("NUS TX notifications %s", tx_notifications_enabled ? "enabled" : "disabled");
}

static ssize_t nus_rx_write(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len,
                            uint16_t offset, uint8_t flags)
{
    (void)conn;
    (void)attr;
    (void)offset;
    (void)flags;

    LOG_DBG("NUS RX: %u bytes", len);

    if ((rx_callback != NULL) && (len > 0U)) {
        rx_callback((const uint8_t *)buf, len);
    }

    return (ssize_t)len;
}

/* ==========================================================================
 * GATT Service Definition
 * ========================================================================== */

BT_GATT_SERVICE_DEFINE(nus_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(nus_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX,
                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                          BT_GATT_PERM_WRITE,
                          NULL, nus_rx_write, NULL),
);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ble_nus_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    tx_notifications_enabled = false;
    rx_callback = NULL;
    is_initialized = true;

    LOG_INF("NUS initialized");
    return APP_OK;
}

app_err_t ble_nus_send(const uint8_t *data, uint16_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    if (!tx_notifications_enabled) {
        return APP_ERR_NOT_INIT;
    }

    /* Get TX characteristic attribute */
    const struct bt_gatt_attr *attr = &nus_svc.attrs[2];

    /* Send in chunks if needed */
    uint16_t offset = 0U;
    while (offset < len) {
        uint16_t chunk_len = MIN(len - offset, NUS_MAX_LEN);

        int err = bt_gatt_notify(NULL, attr, &data[offset], chunk_len);
        if (err < 0) {
            LOG_ERR("NUS notify failed: %d", err);
            return APP_ERR_IO;
        }

        offset += chunk_len;
    }

    return APP_OK;
}

app_err_t ble_nus_send_str(const char *str)
{
    if (str == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    return ble_nus_send((const uint8_t *)str, (uint16_t)strlen(str));
}

app_err_t ble_nus_register_callback(ble_nus_rx_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    rx_callback = callback;
    return APP_OK;
}

bool ble_nus_is_ready(void)
{
    return is_initialized && tx_notifications_enabled;
}
