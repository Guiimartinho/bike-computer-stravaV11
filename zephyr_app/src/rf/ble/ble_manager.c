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
#include "rf/ble_hrs_client.h"
#include "rf/ble_bsc_client.h"

LOG_MODULE_REGISTER(ble_mgr, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Advertising interval (units of 0.625ms) */
#define ADV_INTERVAL_MIN    160U    /* 100ms */
#define ADV_INTERVAL_MAX    240U    /* 150ms */

/** Scan interval and window (units of 0.625ms) */
#define SCAN_INTERVAL       160U    /* 100ms */
#define SCAN_WINDOW         80U     /* 50ms */

/** Scan timeout in seconds (0 = no timeout) */
#define SCAN_TIMEOUT        30U

/** Maximum pending connections */
#define MAX_PENDING_CONN    2U

/** Service UUIDs for filtering */
#define BT_UUID_HRS_VAL     0x180D
#define BT_UUID_CSC_VAL     0x1816

/** Sensor type flags */
#define SENSOR_TYPE_HRS     0x01U
#define SENSOR_TYPE_BSC     0x02U

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current BLE state */
static ble_state_t ble_state = BLE_STATE_IDLE;

/** Current peripheral connection (phone app) */
static struct bt_conn *peripheral_conn;

/** HRS sensor connection */
static struct bt_conn *hrs_conn;

/** BSC sensor connection */
static struct bt_conn *bsc_conn;

/** Connection info */
static ble_conn_info_t conn_info;

/** Event callback */
static ble_event_callback_t event_callback;

/** Initialization flag */
static bool is_initialized;

/** Scanning active flag */
static bool is_scanning;

/** Pending sensor addresses to connect */
typedef struct {
    bt_addr_le_t addr;
    uint8_t sensor_type;
    bool pending;
} pending_sensor_t;

static pending_sensor_t pending_sensors[MAX_PENDING_CONN];

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
 * Scan Helpers
 * ========================================================================== */

/**
 * @brief Check if advertisement contains a specific 16-bit UUID
 */
static bool ad_contains_uuid16(struct bt_data *data, void *user_data)
{
    uint16_t *target_uuid = user_data;
    const uint8_t *uuid_data;
    uint8_t num_uuids;

    if ((data->type != BT_DATA_UUID16_SOME) && (data->type != BT_DATA_UUID16_ALL)) {
        return true; /* Continue parsing */
    }

    num_uuids = data->data_len / sizeof(uint16_t);
    uuid_data = data->data;

    for (uint8_t i = 0U; i < num_uuids; i++) {
        uint16_t uuid = uuid_data[i * 2U] | ((uint16_t)uuid_data[(i * 2U) + 1U] << 8U);
        if (uuid == *target_uuid) {
            *target_uuid = 0U; /* Signal found */
            return false; /* Stop parsing */
        }
    }

    return true; /* Continue parsing */
}

/**
 * @brief Check if device advertises HRS or CSC service
 */
static uint8_t detect_sensor_type(const uint8_t *ad_data, uint8_t ad_len)
{
    uint8_t sensor_type = 0U;
    struct net_buf_simple buf;
    uint16_t uuid;

    net_buf_simple_init_with_data(&buf, (uint8_t *)ad_data, ad_len);

    /* Check for HRS */
    uuid = BT_UUID_HRS_VAL;
    bt_data_parse(&buf, ad_contains_uuid16, &uuid);
    if (uuid == 0U) {
        sensor_type |= SENSOR_TYPE_HRS;
    }

    /* Reset buffer and check for CSC */
    net_buf_simple_init_with_data(&buf, (uint8_t *)ad_data, ad_len);
    uuid = BT_UUID_CSC_VAL;
    bt_data_parse(&buf, ad_contains_uuid16, &uuid);
    if (uuid == 0U) {
        sensor_type |= SENSOR_TYPE_BSC;
    }

    return sensor_type;
}

/**
 * @brief Connect to a discovered sensor
 */
static void connect_to_sensor(const bt_addr_le_t *addr, uint8_t sensor_type)
{
    struct bt_conn *conn;
    int err;

    /* Check if we already have this sensor type connected */
    if (((sensor_type & SENSOR_TYPE_HRS) != 0U) && (hrs_conn != NULL)) {
        LOG_DBG("HRS already connected");
        return;
    }
    if (((sensor_type & SENSOR_TYPE_BSC) != 0U) && (bsc_conn != NULL)) {
        LOG_DBG("BSC already connected");
        return;
    }

    /* Stop scanning before connecting */
    (void)bt_le_scan_stop();
    is_scanning = false;

    struct bt_conn_le_create_param create_param = BT_CONN_LE_CREATE_PARAM_INIT(
        BT_CONN_LE_OPT_NONE,
        BT_GAP_SCAN_FAST_INTERVAL,
        BT_GAP_SCAN_FAST_WINDOW
    );

    struct bt_le_conn_param conn_param = BT_LE_CONN_PARAM_INIT(
        24,  /* min interval: 30ms */
        40,  /* max interval: 50ms */
        0,   /* latency */
        400  /* timeout: 4s */
    );

    err = bt_conn_le_create(addr, &create_param, &conn_param, &conn);
    if (err != 0) {
        LOG_ERR("Failed to create connection (err %d)", err);
        /* Resume scanning */
        (void)ble_manager_start_scan();
        return;
    }

    /* Store pending sensor info */
    for (int i = 0; i < MAX_PENDING_CONN; i++) {
        if (!pending_sensors[i].pending) {
            bt_addr_le_copy(&pending_sensors[i].addr, addr);
            pending_sensors[i].sensor_type = sensor_type;
            pending_sensors[i].pending = true;
            break;
        }
    }

    ble_state = BLE_STATE_CONNECTING;
    LOG_INF("Connecting to sensor (type 0x%02X)", sensor_type);
}

/**
 * @brief Scan callback
 */
static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    uint8_t sensor_type;

    /* Only process connectable advertisements */
    if ((info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) == 0U) {
        return;
    }

    /* Check if this device advertises sensors we care about */
    sensor_type = detect_sensor_type(buf->data, buf->len);

    if (sensor_type == 0U) {
        return; /* Not a sensor we're looking for */
    }

    bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
    LOG_INF("Found sensor: %s (type 0x%02X, RSSI %d)",
            addr_str, sensor_type, info->rssi);

    /* Connect to sensor */
    connect_to_sensor(info->addr, sensor_type);
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

/* ==========================================================================
 * Connection Callbacks
 * ========================================================================== */

static void connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info;
    uint8_t sensor_type = 0U;
    bool is_sensor = false;

    if (err != 0U) {
        LOG_ERR("Connection failed (err %u)", err);
        /* Clear pending and resume scan */
        for (int i = 0; i < MAX_PENDING_CONN; i++) {
            pending_sensors[i].pending = false;
        }
        (void)ble_manager_start_scan();
        return;
    }

    /* Get connection info */
    if (bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Failed to get connection info");
        return;
    }

    /* Check if this is a pending sensor connection */
    for (int i = 0; i < MAX_PENDING_CONN; i++) {
        if (pending_sensors[i].pending &&
            bt_addr_le_cmp(&pending_sensors[i].addr, info.le.dst) == 0) {
            sensor_type = pending_sensors[i].sensor_type;
            pending_sensors[i].pending = false;
            is_sensor = true;
            break;
        }
    }

    if (is_sensor) {
        /* Sensor connection */
        if ((sensor_type & SENSOR_TYPE_HRS) != 0U) {
            hrs_conn = bt_conn_ref(conn);
            ble_hrs_client_on_connect(conn);
            LOG_INF("HRS sensor connected");
        }
        if ((sensor_type & SENSOR_TYPE_BSC) != 0U) {
            bsc_conn = bt_conn_ref(conn);
            ble_bsc_client_on_connect(conn);
            LOG_INF("BSC sensor connected");
        }

        /* Resume scanning for other sensors */
        if ((hrs_conn == NULL) || (bsc_conn == NULL)) {
            (void)ble_manager_start_scan();
        }
    } else {
        /* Peripheral connection (phone app) */
        peripheral_conn = bt_conn_ref(conn);
        (void)memcpy(conn_info.addr, info.le.dst->a.val, sizeof(conn_info.addr));
        conn_info.connected = true;
        ble_state = BLE_STATE_CONNECTED;
        LOG_INF("Peripheral connected");
    }

    if (event_callback != NULL) {
        event_callback(1U, NULL);  /* Event: Connected */
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);

    /* Check which connection was lost */
    if (conn == peripheral_conn) {
        bt_conn_unref(peripheral_conn);
        peripheral_conn = NULL;
        conn_info.connected = false;
        ble_state = BLE_STATE_IDLE;
        LOG_INF("Peripheral disconnected");
        /* Restart advertising */
        (void)ble_manager_start_advertising();
    } else if (conn == hrs_conn) {
        ble_hrs_client_on_disconnect(conn);
        bt_conn_unref(hrs_conn);
        hrs_conn = NULL;
        LOG_INF("HRS sensor disconnected");
        /* Resume scanning to reconnect */
        (void)ble_manager_start_scan();
    } else if (conn == bsc_conn) {
        ble_bsc_client_on_disconnect(conn);
        bt_conn_unref(bsc_conn);
        bsc_conn = NULL;
        LOG_INF("BSC sensor disconnected");
        /* Resume scanning to reconnect */
        (void)ble_manager_start_scan();
    }

    if (event_callback != NULL) {
        event_callback(2U, NULL);  /* Event: Disconnected */
    }
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

    /* Register scan callbacks */
    bt_le_scan_cb_register(&scan_callbacks);

    /* Initialize services */
    err = ble_nus_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("NUS init failed: %d", err);
    }

    err = ble_lns_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("LNS init failed: %d", err);
    }

    /* Initialize sensor clients */
    err = ble_hrs_client_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("HRS client init failed: %d", err);
    }

    err = ble_bsc_client_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_WRN("BSC client init failed: %d", err);
    }

    /* Clear connection info */
    (void)memset(&conn_info, 0, sizeof(conn_info));
    (void)memset(pending_sensors, 0, sizeof(pending_sensors));

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
    int err;

    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (is_scanning) {
        return APP_OK;
    }

    /* Check if we need more sensors */
    if ((hrs_conn != NULL) && (bsc_conn != NULL)) {
        LOG_INF("All sensors connected, not scanning");
        return APP_OK;
    }

    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_ACTIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = SCAN_INTERVAL,
        .window = SCAN_WINDOW,
        .timeout = SCAN_TIMEOUT,
    };

    err = bt_le_scan_start(&scan_param, NULL);
    if (err != 0) {
        LOG_ERR("Scan start failed (err %d)", err);
        return APP_ERR_IO;
    }

    is_scanning = true;
    ble_state = BLE_STATE_SCANNING;
    LOG_INF("Scanning for sensors...");

    return APP_OK;
}

app_err_t ble_manager_stop_scan(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (!is_scanning) {
        return APP_OK;
    }

    int err = bt_le_scan_stop();
    if (err < 0) {
        LOG_ERR("Scan failed to stop (err %d)", err);
        return APP_ERR_IO;
    }

    is_scanning = false;
    if (ble_state == BLE_STATE_SCANNING) {
        ble_state = BLE_STATE_IDLE;
    }
    LOG_INF("Scanning stopped");

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
