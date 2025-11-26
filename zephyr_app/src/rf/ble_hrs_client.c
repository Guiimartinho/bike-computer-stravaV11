/**
 * @file ble_hrs_client.c
 * @brief BLE Heart Rate Service Client implementation
 *
 * GATT client for Heart Rate Service (0x180D)
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "rf/ble_hrs_client.h"

LOG_MODULE_REGISTER(ble_hrs_client, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Heart Rate Service UUID */
#define BT_UUID_HRS_VAL             0x180D

/** Heart Rate Measurement Characteristic UUID */
#define BT_UUID_HRS_MEASUREMENT_VAL 0x2A37

/** Body Sensor Location Characteristic UUID */
#define BT_UUID_HRS_BODY_SENSOR_VAL 0x2A38

/** Heart Rate Control Point UUID */
#define BT_UUID_HRS_CONTROL_VAL     0x2A39

/* UUID declarations */
static struct bt_uuid_16 uuid_hrs = BT_UUID_INIT_16(BT_UUID_HRS_VAL);
static struct bt_uuid_16 uuid_hrs_measurement = BT_UUID_INIT_16(BT_UUID_HRS_MEASUREMENT_VAL);

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Client state */
static hrs_client_state_t client_state = HRS_CLIENT_STATE_IDLE;

/** Current connection */
static struct bt_conn *hrs_conn;

/** Discovery parameters */
static struct bt_gatt_discover_params discover_params;

/** Subscribe parameters */
static struct bt_gatt_subscribe_params subscribe_params;

/** HRS Measurement handle */
static uint16_t hrs_measurement_handle;

/** Current heart rate data */
static hrm_info_t hrm_data;

/** Data callback */
static hrs_data_callback_t data_callback;

/** Connection callback */
static hrs_conn_callback_t conn_callback;

/** Initialization flag */
static bool is_initialized;

/** Mutex for data protection */
static K_MUTEX_DEFINE(hrs_mutex);

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Parse Heart Rate Measurement characteristic value
 */
static void parse_hrm_value(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len < 2U)) {
        return;
    }

    uint8_t flags = data[0];
    uint8_t offset = 1U;

    k_mutex_lock(&hrs_mutex, K_FOREVER);

    /* Heart Rate Value Format */
    if ((flags & 0x01U) != 0U) {
        /* 16-bit heart rate value */
        if (len >= (offset + 2U)) {
            hrm_data.bpm = (uint8_t)((data[offset] | ((uint16_t)data[offset + 1U] << 8U)) & 0xFFU);
            offset += 2U;
        }
    } else {
        /* 8-bit heart rate value */
        hrm_data.bpm = data[offset];
        offset += 1U;
    }

    /* Sensor Contact Status (bits 1-2) */
    /* Not used in this implementation */

    /* Energy Expended Present (bit 3) */
    if ((flags & 0x08U) != 0U) {
        /* Skip energy expended field (2 bytes) */
        offset += 2U;
    }

    /* RR-Interval Present (bit 4) */
    if ((flags & 0x10U) != 0U) {
        if (len >= (offset + 2U)) {
            /* RR-Interval in 1/1024 seconds, convert to ms */
            uint16_t rr_raw = data[offset] | ((uint16_t)data[offset + 1U] << 8U);
            hrm_data.rr_interval = (uint16_t)((rr_raw * 1000U) / 1024U);
        }
    }

    hrm_data.timestamp = k_uptime_get_32();
    hrm_data.connected = true;

    k_mutex_unlock(&hrs_mutex);

    LOG_DBG("HR: %d bpm, RR: %d ms", hrm_data.bpm, hrm_data.rr_interval);

    /* Notify callback */
    if (data_callback != NULL) {
        data_callback(hrm_data.bpm, hrm_data.rr_interval);
    }
}

/**
 * @brief Notification callback for HR Measurement
 */
static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (data == NULL) {
        /* Subscription ended */
        LOG_INF("HRS notifications disabled");
        params->value_handle = 0U;
        client_state = HRS_CLIENT_STATE_CONNECTED;
        return BT_GATT_ITER_STOP;
    }

    parse_hrm_value(data, length);

    return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Subscribe to HR Measurement notifications
 */
static app_err_t subscribe_to_hrm(struct bt_conn *conn)
{
    int err;

    subscribe_params.notify = notify_func;
    subscribe_params.value_handle = hrs_measurement_handle;
    subscribe_params.ccc_handle = 0U; /* Auto-discover CCC */
    subscribe_params.value = BT_GATT_CCC_NOTIFY;

    err = bt_gatt_subscribe(conn, &subscribe_params);
    if (err != 0) {
        LOG_ERR("Subscribe failed (err %d)", err);
        return APP_ERR_IO;
    }

    client_state = HRS_CLIENT_STATE_SUBSCRIBED;
    LOG_INF("Subscribed to HR Measurement notifications");

    return APP_OK;
}

/**
 * @brief GATT discovery callback
 */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (attr == NULL) {
        LOG_INF("Discovery complete");
        (void)memset(params, 0, sizeof(*params));

        if (hrs_measurement_handle != 0U) {
            /* Found the characteristic, subscribe */
            (void)subscribe_to_hrm(conn);
        } else {
            LOG_WRN("HR Measurement characteristic not found");
        }

        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("Discovered attr handle %u", attr->handle);

    if (params->type == BT_GATT_DISCOVER_PRIMARY) {
        /* Found HRS service, now discover characteristics */
        struct bt_gatt_service_val *svc = attr->user_data;

        params->uuid = &uuid_hrs_measurement.uuid;
        params->start_handle = attr->handle + 1U;
        params->end_handle = svc->end_handle;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

        int err = bt_gatt_discover(conn, params);
        if (err != 0) {
            LOG_ERR("Characteristic discovery failed (err %d)", err);
        }

        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc *chrc = attr->user_data;

        if (bt_uuid_cmp(chrc->uuid, &uuid_hrs_measurement.uuid) == 0) {
            hrs_measurement_handle = chrc->value_handle;
            LOG_INF("Found HR Measurement handle: %u", hrs_measurement_handle);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Start HRS service discovery
 */
static app_err_t start_discovery(struct bt_conn *conn)
{
    int err;

    hrs_measurement_handle = 0U;

    discover_params.uuid = &uuid_hrs.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(conn, &discover_params);
    if (err != 0) {
        LOG_ERR("Discovery start failed (err %d)", err);
        return APP_ERR_IO;
    }

    client_state = HRS_CLIENT_STATE_DISCOVERING;
    LOG_INF("HRS discovery started");

    return APP_OK;
}

/* ==========================================================================
 * Connection Callbacks (called by ble_manager)
 * ========================================================================== */

/**
 * @brief Called when connected to a device with HRS
 */
void ble_hrs_client_on_connect(struct bt_conn *conn)
{
    if (!is_initialized) {
        return;
    }

    if (hrs_conn != NULL) {
        LOG_WRN("Already connected to an HRS device");
        return;
    }

    hrs_conn = bt_conn_ref(conn);
    client_state = HRS_CLIENT_STATE_CONNECTED;

    LOG_INF("HRS device connected");

    /* Start discovery */
    (void)start_discovery(conn);

    /* Notify connection callback */
    if (conn_callback != NULL) {
        conn_callback(true);
    }
}

/**
 * @brief Called when disconnected from HRS device
 */
void ble_hrs_client_on_disconnect(struct bt_conn *conn)
{
    if (!is_initialized) {
        return;
    }

    if (hrs_conn != conn) {
        return;
    }

    bt_conn_unref(hrs_conn);
    hrs_conn = NULL;

    hrs_measurement_handle = 0U;
    client_state = HRS_CLIENT_STATE_IDLE;

    k_mutex_lock(&hrs_mutex, K_FOREVER);
    hrm_data.connected = false;
    hrm_data.bpm = 0U;
    k_mutex_unlock(&hrs_mutex);

    LOG_INF("HRS device disconnected");

    /* Notify connection callback */
    if (conn_callback != NULL) {
        conn_callback(false);
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ble_hrs_client_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Initialize data */
    (void)memset(&hrm_data, 0, sizeof(hrm_data));
    hrs_conn = NULL;
    hrs_measurement_handle = 0U;
    client_state = HRS_CLIENT_STATE_IDLE;

    is_initialized = true;
    LOG_INF("HRS client initialized");

    return APP_OK;
}

void ble_hrs_client_register_callback(hrs_data_callback_t callback)
{
    data_callback = callback;
}

void ble_hrs_client_register_conn_callback(hrs_conn_callback_t callback)
{
    conn_callback = callback;
}

bool ble_hrs_client_is_connected(void)
{
    return (client_state == HRS_CLIENT_STATE_SUBSCRIBED) && (hrs_conn != NULL);
}

uint8_t ble_hrs_client_get_bpm(void)
{
    uint8_t bpm;

    k_mutex_lock(&hrs_mutex, K_FOREVER);
    bpm = hrm_data.bpm;
    k_mutex_unlock(&hrs_mutex);

    return bpm;
}

uint16_t ble_hrs_client_get_rr(void)
{
    uint16_t rr;

    k_mutex_lock(&hrs_mutex, K_FOREVER);
    rr = hrm_data.rr_interval;
    k_mutex_unlock(&hrs_mutex);

    return rr;
}

app_err_t ble_hrs_client_get_info(hrm_info_t *info)
{
    if (info == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    k_mutex_lock(&hrs_mutex, K_FOREVER);
    *info = hrm_data;
    k_mutex_unlock(&hrs_mutex);

    return APP_OK;
}
