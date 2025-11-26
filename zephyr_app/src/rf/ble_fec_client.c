/**
 * @file ble_fec_client.c
 * @brief BLE Fitness Machine Service Client implementation
 *
 * GATT client for FTMS (0x1826) - Smart trainer control
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "rf/ble_fec_client.h"

LOG_MODULE_REGISTER(ble_fec_client, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Fitness Machine Service UUID */
#define BT_UUID_FTMS_VAL                0x1826

/** Indoor Bike Data Characteristic UUID */
#define BT_UUID_INDOOR_BIKE_DATA_VAL    0x2AD2

/** Fitness Machine Control Point UUID */
#define BT_UUID_FTMS_CONTROL_VAL        0x2AD9

/** Fitness Machine Status UUID */
#define BT_UUID_FTMS_STATUS_VAL         0x2ADA

/* Control Point Op Codes */
#define FTMS_OP_REQUEST_CONTROL         0x00
#define FTMS_OP_RESET                   0x01
#define FTMS_OP_SET_TARGET_POWER        0x05
#define FTMS_OP_SET_TARGET_RESISTANCE   0x04
#define FTMS_OP_SET_SIMULATION          0x11
#define FTMS_OP_START                   0x07
#define FTMS_OP_STOP                    0x08

/* Indoor Bike Data flags */
#define IBD_FLAG_SPEED_PRESENT          0x0001
#define IBD_FLAG_CADENCE_PRESENT        0x0002
#define IBD_FLAG_POWER_PRESENT          0x0040
#define IBD_FLAG_ELAPSED_TIME_PRESENT   0x0080

/* UUID declarations */
static struct bt_uuid_16 uuid_ftms = BT_UUID_INIT_16(BT_UUID_FTMS_VAL);
static struct bt_uuid_16 uuid_bike_data = BT_UUID_INIT_16(BT_UUID_INDOOR_BIKE_DATA_VAL);
static struct bt_uuid_16 uuid_control = BT_UUID_INIT_16(BT_UUID_FTMS_CONTROL_VAL);

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Client state */
static fec_client_state_t client_state = FEC_CLIENT_STATE_IDLE;

/** Current connection */
static struct bt_conn *fec_conn;

/** Discovery parameters */
static struct bt_gatt_discover_params discover_params;

/** Subscribe parameters */
static struct bt_gatt_subscribe_params subscribe_params;

/** Write parameters for control point */
static struct bt_gatt_write_params write_params;

/** Characteristic handles */
static uint16_t bike_data_handle;
static uint16_t control_handle;

/** Current FEC data */
static fec_data_t fec_data;

/** Data callback */
static fec_data_callback_t data_callback;

/** Connection callback */
static fec_conn_callback_t conn_callback;

/** Initialization flag */
static bool is_initialized;

/** Control buffer */
static uint8_t control_buf[16];

/** Mutex for data protection */
static K_MUTEX_DEFINE(fec_mutex);

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Parse Indoor Bike Data characteristic
 */
static void parse_bike_data(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len < 2U)) {
        return;
    }

    uint16_t flags = data[0] | ((uint16_t)data[1] << 8U);
    uint8_t offset = 2U;

    k_mutex_lock(&fec_mutex, K_FOREVER);

    /* Speed (0.01 km/h) - always present if not flagged absent */
    if ((flags & IBD_FLAG_SPEED_PRESENT) == 0U) {
        if (len >= (offset + 2U)) {
            fec_data.speed = data[offset] | ((uint16_t)data[offset + 1U] << 8U);
            offset += 2U;
        }
    }

    /* Average speed - skip */
    if ((flags & 0x0002U) != 0U) {
        offset += 2U;
    }

    /* Instantaneous cadence */
    if ((flags & IBD_FLAG_CADENCE_PRESENT) != 0U) {
        if (len >= (offset + 2U)) {
            /* Cadence is in 0.5 RPM units */
            uint16_t cad_raw = data[offset] | ((uint16_t)data[offset + 1U] << 8U);
            fec_data.cadence = (uint8_t)(cad_raw / 2U);
            offset += 2U;
        }
    }

    /* Average cadence - skip */
    if ((flags & 0x0008U) != 0U) {
        offset += 2U;
    }

    /* Total distance - skip */
    if ((flags & 0x0010U) != 0U) {
        offset += 3U;
    }

    /* Resistance level - skip */
    if ((flags & 0x0020U) != 0U) {
        offset += 2U;
    }

    /* Instantaneous power */
    if ((flags & IBD_FLAG_POWER_PRESENT) != 0U) {
        if (len >= (offset + 2U)) {
            fec_data.power = data[offset] | ((uint16_t)data[offset + 1U] << 8U);
            offset += 2U;
        }
    }

    /* Average power - skip */
    if ((flags & 0x0080U) != 0U) {
        offset += 2U;
    }

    /* Expended energy - skip */
    if ((flags & 0x0100U) != 0U) {
        offset += 4U;
    }

    /* Heart rate - skip */
    if ((flags & 0x0200U) != 0U) {
        offset += 1U;
    }

    /* Metabolic equivalent - skip */
    if ((flags & 0x0400U) != 0U) {
        offset += 1U;
    }

    /* Elapsed time */
    if ((flags & IBD_FLAG_ELAPSED_TIME_PRESENT) != 0U) {
        if (len >= (offset + 2U)) {
            fec_data.elapsed_time = data[offset] | ((uint16_t)data[offset + 1U] << 8U);
        }
    }

    fec_data.timestamp = k_uptime_get_32();
    fec_data.connected = true;
    fec_data.status = TRAINER_STATUS_IN_USE;

    k_mutex_unlock(&fec_mutex);

    LOG_DBG("FEC: %u W, %u RPM, %u.%02u km/h",
            fec_data.power, fec_data.cadence,
            fec_data.speed / 100, fec_data.speed % 100);

    /* Notify callback */
    if (data_callback != NULL) {
        data_callback(&fec_data);
    }
}

/**
 * @brief Notification callback for Indoor Bike Data
 */
static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (data == NULL) {
        LOG_INF("FTMS notifications disabled");
        params->value_handle = 0U;
        client_state = FEC_CLIENT_STATE_CONNECTED;
        return BT_GATT_ITER_STOP;
    }

    parse_bike_data(data, length);

    return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Write callback for control point
 */
static void write_cb(struct bt_conn *conn, uint8_t err,
                     struct bt_gatt_write_params *params)
{
    if (err != 0U) {
        LOG_ERR("Control write failed: %u", err);
    } else {
        LOG_DBG("Control write success");
    }
}

/**
 * @brief Send control point command
 */
static app_err_t send_control_cmd(const uint8_t *cmd, uint8_t len)
{
    if (fec_conn == NULL || control_handle == 0U) {
        return APP_ERR_NOT_FOUND;
    }

    memcpy(control_buf, cmd, len);

    write_params.func = write_cb;
    write_params.handle = control_handle;
    write_params.offset = 0U;
    write_params.data = control_buf;
    write_params.length = len;

    int err = bt_gatt_write(fec_conn, &write_params);
    if (err != 0) {
        LOG_ERR("Control write error: %d", err);
        return APP_ERR_IO;
    }

    return APP_OK;
}

/**
 * @brief Subscribe to Indoor Bike Data notifications
 */
static app_err_t subscribe_to_data(struct bt_conn *conn)
{
    int err;

    subscribe_params.notify = notify_func;
    subscribe_params.value_handle = bike_data_handle;
    subscribe_params.ccc_handle = 0U;
    subscribe_params.value = BT_GATT_CCC_NOTIFY;

    err = bt_gatt_subscribe(conn, &subscribe_params);
    if (err != 0) {
        LOG_ERR("Subscribe failed: %d", err);
        return APP_ERR_IO;
    }

    client_state = FEC_CLIENT_STATE_SUBSCRIBED;
    LOG_INF("Subscribed to FTMS notifications");

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
        LOG_INF("FTMS Discovery complete");
        (void)memset(params, 0, sizeof(*params));

        if (bike_data_handle != 0U) {
            (void)subscribe_to_data(conn);
        } else {
            LOG_WRN("Indoor Bike Data characteristic not found");
        }

        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("Discovered attr handle %u", attr->handle);

    if (params->type == BT_GATT_DISCOVER_PRIMARY) {
        struct bt_gatt_service_val *svc = attr->user_data;

        params->uuid = NULL;  /* Discover all characteristics */
        params->start_handle = attr->handle + 1U;
        params->end_handle = svc->end_handle;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

        int err = bt_gatt_discover(conn, params);
        if (err != 0) {
            LOG_ERR("Characteristic discovery failed: %d", err);
        }

        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc *chrc = attr->user_data;

        if (bt_uuid_cmp(chrc->uuid, &uuid_bike_data.uuid) == 0) {
            bike_data_handle = chrc->value_handle;
            LOG_INF("Found Indoor Bike Data: %u", bike_data_handle);
        } else if (bt_uuid_cmp(chrc->uuid, &uuid_control.uuid) == 0) {
            control_handle = chrc->value_handle;
            LOG_INF("Found Control Point: %u", control_handle);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Start FTMS service discovery
 */
static app_err_t start_discovery(struct bt_conn *conn)
{
    int err;

    bike_data_handle = 0U;
    control_handle = 0U;

    discover_params.uuid = &uuid_ftms.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(conn, &discover_params);
    if (err != 0) {
        LOG_ERR("Discovery start failed: %d", err);
        return APP_ERR_IO;
    }

    client_state = FEC_CLIENT_STATE_DISCOVERING;
    LOG_INF("FTMS discovery started");

    return APP_OK;
}

/* ==========================================================================
 * Connection Callbacks
 * ========================================================================== */

void ble_fec_client_on_connect(struct bt_conn *conn)
{
    if (!is_initialized) {
        return;
    }

    if (fec_conn != NULL) {
        LOG_WRN("Already connected to a trainer");
        return;
    }

    fec_conn = bt_conn_ref(conn);
    client_state = FEC_CLIENT_STATE_CONNECTED;

    LOG_INF("Trainer connected");

    (void)start_discovery(conn);

    if (conn_callback != NULL) {
        conn_callback(true);
    }
}

void ble_fec_client_on_disconnect(struct bt_conn *conn)
{
    if (!is_initialized) {
        return;
    }

    if (fec_conn != conn) {
        return;
    }

    bt_conn_unref(fec_conn);
    fec_conn = NULL;

    bike_data_handle = 0U;
    control_handle = 0U;
    client_state = FEC_CLIENT_STATE_IDLE;

    k_mutex_lock(&fec_mutex, K_FOREVER);
    fec_data.connected = false;
    fec_data.power = 0U;
    fec_data.status = TRAINER_STATUS_UNKNOWN;
    k_mutex_unlock(&fec_mutex);

    LOG_INF("Trainer disconnected");

    if (conn_callback != NULL) {
        conn_callback(false);
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ble_fec_client_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    (void)memset(&fec_data, 0, sizeof(fec_data));
    fec_conn = NULL;
    bike_data_handle = 0U;
    control_handle = 0U;
    client_state = FEC_CLIENT_STATE_IDLE;

    is_initialized = true;
    LOG_INF("FEC client initialized");

    return APP_OK;
}

void ble_fec_client_register_callback(fec_data_callback_t callback)
{
    data_callback = callback;
}

void ble_fec_client_register_conn_callback(fec_conn_callback_t callback)
{
    conn_callback = callback;
}

bool ble_fec_client_is_connected(void)
{
    return (client_state >= FEC_CLIENT_STATE_SUBSCRIBED) && (fec_conn != NULL);
}

app_err_t ble_fec_client_get_info(fec_info_t *info)
{
    if (info == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    k_mutex_lock(&fec_mutex, K_FOREVER);
    info->power = fec_data.power;
    info->cadence = fec_data.cadence;
    info->grade = fec_data.grade;
    info->el_time = (uint16_t)(fec_data.elapsed_time & 0xFFFFU);
    info->connected = fec_data.connected;
    k_mutex_unlock(&fec_mutex);

    return APP_OK;
}

app_err_t ble_fec_client_get_data(fec_data_t *data)
{
    if (data == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    k_mutex_lock(&fec_mutex, K_FOREVER);
    *data = fec_data;
    k_mutex_unlock(&fec_mutex);

    return APP_OK;
}

app_err_t ble_fec_client_set_grade(int8_t grade)
{
    /* Clamp grade */
    if (grade > FEC_MAX_GRADE) {
        grade = FEC_MAX_GRADE;
    }
    if (grade < FEC_MIN_GRADE) {
        grade = FEC_MIN_GRADE;
    }

    /* FTMS simulation params: wind, grade, crr, cw */
    /* Grade is in 0.01% resolution, range -100% to +100% */
    int16_t grade_ftms = grade * 100;

    uint8_t cmd[] = {
        FTMS_OP_SET_SIMULATION,
        0, 0,                           /* Wind speed (int16, m/s * 1000) */
        (uint8_t)(grade_ftms & 0xFF),   /* Grade low byte */
        (uint8_t)(grade_ftms >> 8),     /* Grade high byte */
        50,                             /* Crr * 10000 (0.005) */
        50                              /* Cw * 100 (0.5) */
    };

    k_mutex_lock(&fec_mutex, K_FOREVER);
    fec_data.grade = grade;
    fec_data.mode = FEC_MODE_SIMULATION;
    k_mutex_unlock(&fec_mutex);

    return send_control_cmd(cmd, sizeof(cmd));
}

app_err_t ble_fec_client_set_target_power(uint16_t power)
{
    uint8_t cmd[] = {
        FTMS_OP_SET_TARGET_POWER,
        (uint8_t)(power & 0xFF),
        (uint8_t)(power >> 8)
    };

    k_mutex_lock(&fec_mutex, K_FOREVER);
    fec_data.target_power = power;
    fec_data.mode = FEC_MODE_ERG;
    k_mutex_unlock(&fec_mutex);

    return send_control_cmd(cmd, sizeof(cmd));
}

app_err_t ble_fec_client_set_resistance(uint8_t level)
{
    if (level > 100U) {
        level = 100U;
    }

    uint8_t cmd[] = {
        FTMS_OP_SET_TARGET_RESISTANCE,
        (uint8_t)(level * 10)  /* 0.1 resolution */
    };

    k_mutex_lock(&fec_mutex, K_FOREVER);
    fec_data.resistance = level;
    fec_data.mode = FEC_MODE_RESISTANCE;
    k_mutex_unlock(&fec_mutex);

    return send_control_cmd(cmd, sizeof(cmd));
}

app_err_t ble_fec_client_set_mode(fec_mode_t mode)
{
    k_mutex_lock(&fec_mutex, K_FOREVER);
    fec_data.mode = mode;
    k_mutex_unlock(&fec_mutex);

    return APP_OK;
}

app_err_t ble_fec_client_start(void)
{
    /* Request control first */
    uint8_t cmd_req[] = { FTMS_OP_REQUEST_CONTROL };
    app_err_t err = send_control_cmd(cmd_req, sizeof(cmd_req));
    if (err != APP_OK) {
        return err;
    }

    k_msleep(100);

    /* Then start */
    uint8_t cmd_start[] = { FTMS_OP_START, 0x01 }; /* Resume */
    err = send_control_cmd(cmd_start, sizeof(cmd_start));
    if (err == APP_OK) {
        client_state = FEC_CLIENT_STATE_CONTROLLING;
    }

    return err;
}

app_err_t ble_fec_client_stop(void)
{
    uint8_t cmd[] = { FTMS_OP_STOP, 0x01 }; /* Stop */
    app_err_t err = send_control_cmd(cmd, sizeof(cmd));

    if (err == APP_OK && client_state == FEC_CLIENT_STATE_CONTROLLING) {
        client_state = FEC_CLIENT_STATE_SUBSCRIBED;
    }

    return err;
}
