/**
 * @file ble_komoot_client.c
 * @brief BLE Komoot Navigation Service Client implementation
 *
 * GATT client for Komoot BLE Connect navigation service.
 * Implements discovery, subscription, and notification handling.
 *
 * Follows MISRA C:2012 guidelines.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "rf/ble_komoot_client.h"

LOG_MODULE_REGISTER(ble_komoot, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Maximum callbacks supported */
#define MAX_CALLBACKS   2U

/** Komoot vendor base UUID: 71C1E128-D92F-4FA8-A2B2-0F171DB3436C */
static const struct bt_uuid_128 komoot_base_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x71C1E128, 0xD92F, 0x4FA8, 0xA2B2, 0x0F171DB3436C)
);

/** Navigation characteristic UUID */
static struct bt_uuid_128 komoot_nav_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x503DD605, 0x9BCB, 0x4F6E, 0xB235, 0xFAFABE2A35ED)
);

/* ==========================================================================
 * Direction Name Strings
 * ========================================================================== */

static const char *direction_names[KOMOOT_DIR_MAX] = {
    [KOMOOT_DIR_NONE]             = "None",
    [KOMOOT_DIR_STRAIGHT]         = "Straight",
    [KOMOOT_DIR_START]            = "Start",
    [KOMOOT_DIR_FINISH]           = "Finish",
    [KOMOOT_DIR_SLIGHT_LEFT]      = "Slight Left",
    [KOMOOT_DIR_LEFT]             = "Left",
    [KOMOOT_DIR_SHARP_LEFT]       = "Sharp Left",
    [KOMOOT_DIR_SLIGHT_RIGHT]     = "Slight Right",
    [KOMOOT_DIR_RIGHT]            = "Right",
    [KOMOOT_DIR_SHARP_RIGHT]      = "Sharp Right",
    [KOMOOT_DIR_FORK_LEFT]        = "Fork Left",
    [KOMOOT_DIR_FORK_RIGHT]       = "Fork Right",
    [KOMOOT_DIR_U_TURN]           = "U-Turn",
    [KOMOOT_DIR_U_TURN_LEFT]      = "U-Turn L",
    [KOMOOT_DIR_U_TURN_RIGHT]     = "U-Turn R",
    [KOMOOT_DIR_ROUNDABOUT_EXIT1] = "RAB Exit 1",
    [KOMOOT_DIR_ROUNDABOUT_EXIT2] = "RAB Exit 2",
    [KOMOOT_DIR_ROUNDABOUT_EXIT3] = "RAB Exit 3",
    [KOMOOT_DIR_ROUNDABOUT_CCW1]  = "RAB CCW 1",
    [KOMOOT_DIR_ROUNDABOUT_CCW2]  = "RAB CCW 2",
    [KOMOOT_DIR_ROUNDABOUT_CCW3]  = "RAB CCW 3",
    [KOMOOT_DIR_ROUNDABOUT_FALL]  = "Roundabout",
    [KOMOOT_DIR_OUT_OF_ROUTE]     = "Off Route",
    [KOMOOT_DIR_FERRY]            = "Ferry"
};

/* ==========================================================================
 * Navigation Icons (32x32 1-bit bitmaps)
 * ========================================================================== */

/** Arrow straight icon */
static const uint8_t icon_straight[128] = {
    0x00, 0x01, 0x80, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x07, 0xE0, 0x00, 0x00, 0x0F, 0xF0, 0x00,
    0x00, 0x1F, 0xF8, 0x00, 0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x7F, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00
};

/** Arrow right icon */
static const uint8_t icon_right[128] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
    0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x1C, 0x00,
    0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x7C, 0x00,
    0x00, 0x00, 0xFC, 0x00, 0x00, 0x01, 0xFC, 0x00,
    0x00, 0x03, 0xFC, 0x00, 0x0F, 0xFF, 0xFC, 0x00,
    0x0F, 0xFF, 0xFC, 0x00, 0x0F, 0xFF, 0xFC, 0x00,
    0x0F, 0xFF, 0xFC, 0x00, 0x00, 0x03, 0xFC, 0x00,
    0x00, 0x01, 0xFC, 0x00, 0x00, 0x00, 0xFC, 0x00,
    0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x3C, 0x00,
    0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x0C, 0x00,
    0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/** Arrow left icon */
static const uint8_t icon_left[128] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00,
    0x00, 0x30, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
    0x00, 0x3C, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00,
    0x00, 0x3F, 0x00, 0x00, 0x00, 0x3F, 0x80, 0x00,
    0x00, 0x3F, 0xC0, 0x00, 0x00, 0x3F, 0xFF, 0xF0,
    0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x3F, 0xFF, 0xF0,
    0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x3F, 0xC0, 0x00,
    0x00, 0x3F, 0x80, 0x00, 0x00, 0x3F, 0x00, 0x00,
    0x00, 0x3E, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00,
    0x00, 0x38, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00,
    0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/** U-turn icon */
static const uint8_t icon_uturn[128] = {
    0x00, 0x0F, 0xF0, 0x00, 0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x7F, 0xFE, 0x00, 0x00, 0xFC, 0x3F, 0x00,
    0x01, 0xF0, 0x0F, 0x80, 0x01, 0xE0, 0x07, 0x80,
    0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0,
    0x03, 0xC0, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0,
    0x00, 0x30, 0x03, 0xC0, 0x00, 0x38, 0x03, 0xC0,
    0x00, 0x3C, 0x03, 0xC0, 0x00, 0x3E, 0x03, 0xC0,
    0x0F, 0xFF, 0x03, 0xC0, 0x0F, 0xFF, 0x83, 0xC0,
    0x0F, 0xFF, 0x03, 0xC0, 0x00, 0x3E, 0x03, 0xC0,
    0x00, 0x3C, 0x03, 0xC0, 0x00, 0x38, 0x03, 0xC0,
    0x00, 0x30, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0,
    0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0,
    0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0,
    0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0,
    0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0,
    0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0
};

/** Finish flag icon */
static const uint8_t icon_finish[128] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
    0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00
};

/** Roundabout icon */
static const uint8_t icon_roundabout[128] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00,
    0x00, 0x1F, 0xF8, 0x00, 0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x78, 0x1E, 0x00, 0x00, 0xE0, 0x07, 0x00,
    0x01, 0xC0, 0x03, 0x80, 0x01, 0x80, 0x01, 0x80,
    0x03, 0x80, 0x01, 0xC0, 0x03, 0x00, 0x00, 0xC0,
    0x03, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x10, 0xC0,
    0x03, 0x00, 0x38, 0xC0, 0x03, 0x00, 0x7C, 0xC0,
    0x03, 0x00, 0xFE, 0xC0, 0x03, 0x01, 0xFF, 0xC0,
    0x03, 0x00, 0xFE, 0x00, 0x03, 0x00, 0x7C, 0x00,
    0x03, 0x00, 0x38, 0x00, 0x03, 0x00, 0x10, 0x00,
    0x03, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00,
    0x01, 0xC0, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00,
    0x00, 0x78, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xC0,
    0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x07, 0xFF, 0xC0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Icon lookup table */
static const uint8_t *icon_table[KOMOOT_DIR_MAX] = {
    [KOMOOT_DIR_NONE]             = NULL,
    [KOMOOT_DIR_STRAIGHT]         = icon_straight,
    [KOMOOT_DIR_START]            = icon_straight,
    [KOMOOT_DIR_FINISH]           = icon_finish,
    [KOMOOT_DIR_SLIGHT_LEFT]      = icon_left,
    [KOMOOT_DIR_LEFT]             = icon_left,
    [KOMOOT_DIR_SHARP_LEFT]       = icon_left,
    [KOMOOT_DIR_SLIGHT_RIGHT]     = icon_right,
    [KOMOOT_DIR_RIGHT]            = icon_right,
    [KOMOOT_DIR_SHARP_RIGHT]      = icon_right,
    [KOMOOT_DIR_FORK_LEFT]        = icon_left,
    [KOMOOT_DIR_FORK_RIGHT]       = icon_right,
    [KOMOOT_DIR_U_TURN]           = icon_uturn,
    [KOMOOT_DIR_U_TURN_LEFT]      = icon_uturn,
    [KOMOOT_DIR_U_TURN_RIGHT]     = icon_uturn,
    [KOMOOT_DIR_ROUNDABOUT_EXIT1] = icon_roundabout,
    [KOMOOT_DIR_ROUNDABOUT_EXIT2] = icon_roundabout,
    [KOMOOT_DIR_ROUNDABOUT_EXIT3] = icon_roundabout,
    [KOMOOT_DIR_ROUNDABOUT_CCW1]  = icon_roundabout,
    [KOMOOT_DIR_ROUNDABOUT_CCW2]  = icon_roundabout,
    [KOMOOT_DIR_ROUNDABOUT_CCW3]  = icon_roundabout,
    [KOMOOT_DIR_ROUNDABOUT_FALL]  = icon_roundabout,
    [KOMOOT_DIR_OUT_OF_ROUTE]     = NULL,
    [KOMOOT_DIR_FERRY]            = NULL
};

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current navigation data */
static komoot_nav_t nav_data;

/** Client state */
static komoot_client_state_t client_state = KOMOOT_STATE_IDLE;

/** Current connection */
static struct bt_conn *komoot_conn;

/** Registered callbacks */
static komoot_nav_callback_t callbacks[MAX_CALLBACKS];
static uint8_t callback_count;

/** Discovery parameters */
static struct bt_gatt_discover_params discover_params;

/** Subscribe parameters */
static struct bt_gatt_subscribe_params subscribe_params;

/** Characteristic handle */
static uint16_t nav_handle;

/** Module initialized flag */
static bool initialized;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Notify all registered callbacks
 */
static void notify_callbacks(void)
{
    for (uint8_t i = 0U; i < callback_count; i++) {
        if (callbacks[i] != NULL) {
            callbacks[i](&nav_data);
        }
    }
}

/**
 * @brief Parse navigation notification data
 * @param data Notification data
 * @param length Data length
 */
static void parse_navigation(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length < 5U)) {
        return;
    }

    /* Komoot navigation format:
     * Byte 0: Direction (0-31)
     * Bytes 1-4: Distance in meters (little-endian uint32)
     * Bytes 5+: Street name (optional, UTF-8)
     */

    uint8_t dir = data[0];
    if (dir >= (uint8_t)KOMOOT_DIR_MAX) {
        dir = (uint8_t)KOMOOT_DIR_NONE;
    }

    nav_data.direction = (komoot_direction_t)dir;
    nav_data.distance = (uint32_t)data[1] |
                       ((uint32_t)data[2] << 8) |
                       ((uint32_t)data[3] << 16) |
                       ((uint32_t)data[4] << 24);

    /* Parse street name if present */
    if (length > 5U) {
        uint16_t name_len = length - 5U;
        if (name_len >= sizeof(nav_data.street_name)) {
            name_len = sizeof(nav_data.street_name) - 1U;
        }
        (void)memcpy(nav_data.street_name, &data[5], name_len);
        nav_data.street_name[name_len] = '\0';
    } else {
        nav_data.street_name[0] = '\0';
    }

    nav_data.is_updated = true;
    nav_data.timestamp = k_uptime_get_32();

    LOG_INF("Komoot nav: dir=%s dist=%um",
            direction_names[nav_data.direction],
            nav_data.distance);

    notify_callbacks();
}

/**
 * @brief Notification callback
 */
static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (data == NULL) {
        LOG_INF("Komoot unsubscribed");
        params->value_handle = 0U;
        client_state = KOMOOT_STATE_CONNECTED;
        return BT_GATT_ITER_STOP;
    }

    parse_navigation((const uint8_t *)data, length);

    return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Discovery callback
 */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (attr == NULL) {
        LOG_INF("Komoot discovery complete");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("Discovered attr handle %u", attr->handle);

    if (bt_uuid_cmp(params->uuid, (struct bt_uuid *)&komoot_base_uuid) == 0) {
        /* Found service, now find characteristic */
        LOG_INF("Komoot service found");

        discover_params.uuid = (struct bt_uuid *)&komoot_nav_uuid;
        discover_params.start_handle = attr->handle + 1U;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        int err = bt_gatt_discover(conn, &discover_params);
        if (err != 0) {
            LOG_ERR("Char discovery failed: %d", err);
        }
    } else if (bt_uuid_cmp(params->uuid, (struct bt_uuid *)&komoot_nav_uuid) == 0) {
        /* Found characteristic, subscribe to notifications */
        LOG_INF("Komoot nav characteristic found");

        nav_handle = bt_gatt_attr_value_handle(attr);

        subscribe_params.notify = notify_func;
        subscribe_params.value_handle = nav_handle;
        subscribe_params.ccc_handle = nav_handle + 1U;  /* CCC is typically next handle */
        subscribe_params.value = BT_GATT_CCC_NOTIFY;

        int err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err != 0) {
            LOG_ERR("Subscribe failed: %d", err);
        } else {
            client_state = KOMOOT_STATE_SUBSCRIBED;
            nav_data.is_connected = true;
            LOG_INF("Subscribed to Komoot notifications");
        }
    }

    return BT_GATT_ITER_STOP;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ble_komoot_client_init(void)
{
    if (initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    (void)memset(&nav_data, 0, sizeof(nav_data));
    (void)memset(callbacks, 0, sizeof(callbacks));
    callback_count = 0U;
    client_state = KOMOOT_STATE_IDLE;
    komoot_conn = NULL;
    nav_handle = 0U;

    initialized = true;
    LOG_INF("Komoot client initialized");

    return APP_OK;
}

app_err_t ble_komoot_client_register_callback(komoot_nav_callback_t callback)
{
    if (callback == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    if (callback_count >= MAX_CALLBACKS) {
        return APP_ERR_NO_MEM;
    }

    callbacks[callback_count] = callback;
    callback_count++;

    return APP_OK;
}

app_err_t ble_komoot_client_get_nav(komoot_nav_t *nav)
{
    if (nav == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    if (!nav_data.is_connected && !nav_data.is_updated) {
        return APP_ERR_NOT_FOUND;
    }

    (void)memcpy(nav, &nav_data, sizeof(komoot_nav_t));

    return APP_OK;
}

bool ble_komoot_client_is_connected(void)
{
    return (client_state == KOMOOT_STATE_SUBSCRIBED);
}

void ble_komoot_client_clear_update(void)
{
    nav_data.is_updated = false;
}

const uint8_t *ble_komoot_get_icon(komoot_direction_t direction)
{
    if (direction >= KOMOOT_DIR_MAX) {
        return NULL;
    }

    return icon_table[direction];
}

const char *ble_komoot_get_direction_name(komoot_direction_t direction)
{
    if (direction >= KOMOOT_DIR_MAX) {
        return "Unknown";
    }

    return direction_names[direction];
}

void ble_komoot_client_on_connect(struct bt_conn *conn)
{
    if (!initialized || (conn == NULL)) {
        return;
    }

    LOG_INF("Starting Komoot service discovery");

    komoot_conn = bt_conn_ref(conn);
    client_state = KOMOOT_STATE_DISCOVERING;

    discover_params.uuid = (struct bt_uuid *)&komoot_base_uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err != 0) {
        LOG_ERR("Discovery start failed: %d", err);
        client_state = KOMOOT_STATE_IDLE;
        bt_conn_unref(komoot_conn);
        komoot_conn = NULL;
    }
}

void ble_komoot_client_on_disconnect(struct bt_conn *conn)
{
    if (komoot_conn == conn) {
        LOG_INF("Komoot disconnected");

        bt_conn_unref(komoot_conn);
        komoot_conn = NULL;
        client_state = KOMOOT_STATE_IDLE;
        nav_data.is_connected = false;
        nav_handle = 0U;
    }
}
