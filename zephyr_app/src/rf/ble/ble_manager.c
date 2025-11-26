/**
 * @file ble_manager.c
 * @brief BLE Manager implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "rf/ble_manager.h"
#include "rf/ble_nus.h"
#include "rf/ble_lns.h"

LOG_MODULE_REGISTER(ble_mgr, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Advertising interval (units of 0.625ms) */
#define ADV_INTERVAL_MIN    160U    /* 100ms */
#define ADV_INTERVAL_MAX    240U    /* 150ms */

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current BLE state */
static ble_state_t ble_state = BLE_STATE_IDLE;

/** Current connection */
static struct bt_conn *current_conn;

/** Connection info */
static ble_conn_info_t conn_info;

/** Event callback */
static ble_event_callback_t event_callback;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Advertising Data
 * ========================================================================== */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
        BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),     /* Battery Service */
        BT_UUID_16_ENCODE(BT_UUID_DIS_VAL),     /* Device Information */
    ),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ==========================================================================
 * Connection Callbacks
 * ========================================================================== */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0U) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    ble_state = BLE_STATE_CONNECTED;

    /* Get connection info */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0) {
        (void)memcpy(conn_info.addr, info.le.dst->a.val, sizeof(conn_info.addr));
        conn_info.connected = true;
    }

    LOG_INF("Connected");

    if (event_callback != NULL) {
        event_callback(1U, NULL);  /* Event: Connected */
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);

    if (current_conn != NULL) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    conn_info.connected = false;
    ble_state = BLE_STATE_IDLE;

    if (event_callback != NULL) {
        event_callback(2U, NULL);  /* Event: Disconnected */
    }

    /* Restart advertising */
    (void)ble_manager_start_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ble_manager_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Enable Bluetooth */
    int err = bt_enable(NULL);
    if (err < 0) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return APP_ERR_NOT_INIT;
    }

    LOG_INF("Bluetooth initialized");

    /* Initialize services */
    err = ble_nus_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("NUS init failed: %d", err);
    }

    err = ble_lns_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("LNS init failed: %d", err);
    }

    /* Clear connection info */
    (void)memset(&conn_info, 0, sizeof(conn_info));

    is_initialized = true;
    LOG_INF("BLE Manager initialized");

    return APP_OK;
}

app_err_t ble_manager_start_advertising(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (ble_state == BLE_STATE_ADVERTISING) {
        return APP_OK;
    }

    if (ble_state == BLE_STATE_CONNECTED) {
        return APP_ERR_BUSY;
    }

    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0U,
        .secondary_max_skip = 0U,
        .options = BT_LE_ADV_OPT_CONN,
        .interval_min = ADV_INTERVAL_MIN,
        .interval_max = ADV_INTERVAL_MAX,
        .peer = NULL,
    };

    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err < 0) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return APP_ERR_IO;
    }

    ble_state = BLE_STATE_ADVERTISING;
    LOG_INF("Advertising started");

    return APP_OK;
}

app_err_t ble_manager_stop_advertising(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (ble_state != BLE_STATE_ADVERTISING) {
        return APP_OK;
    }

    int err = bt_le_adv_stop();
    if (err < 0) {
        LOG_ERR("Advertising failed to stop (err %d)", err);
        return APP_ERR_IO;
    }

    ble_state = BLE_STATE_IDLE;
    LOG_INF("Advertising stopped");

    return APP_OK;
}

app_err_t ble_manager_start_scan(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* TODO: Implement scanning for HRM, BSC sensors */
    LOG_WRN("Scanning not implemented");

    return APP_ERR_NOT_INIT;
}

app_err_t ble_manager_stop_scan(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    int err = bt_le_scan_stop();
    if (err < 0) {
        LOG_ERR("Scan failed to stop (err %d)", err);
        return APP_ERR_IO;
    }

    return APP_OK;
}

ble_state_t ble_manager_get_state(void)
{
    return ble_state;
}

bool ble_manager_is_connected(void)
{
    return (ble_state == BLE_STATE_CONNECTED) && (current_conn != NULL);
}

app_err_t ble_manager_get_conn_info(ble_conn_info_t *info)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (info == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *info = conn_info;
    return APP_OK;
}

app_err_t ble_manager_disconnect(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (current_conn == NULL) {
        return APP_ERR_NOT_FOUND;
    }

    int err = bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    if (err < 0) {
        LOG_ERR("Disconnect failed (err %d)", err);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t ble_manager_register_callback(ble_event_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    event_callback = callback;
    return APP_OK;
}

app_err_t ble_manager_nus_send(const uint8_t *data, uint16_t len)
{
    return ble_nus_send(data, len);
}

void ble_manager_update_battery(uint8_t level)
{
    if (!is_initialized) {
        return;
    }

    int err = bt_bas_set_battery_level(level);
    if (err < 0) {
        LOG_WRN("Failed to update battery level: %d", err);
    }
}
