/**
 * @file ble_lns.c
 * @brief Location and Navigation Service implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "rf/ble_lns.h"

LOG_MODULE_REGISTER(ble_lns, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** LNS Service UUID */
#define BT_UUID_LNS_VAL         0x1819U

/** LNS Location and Speed UUID */
#define BT_UUID_LNS_LOC_SPEED_VAL   0x2A67U

/** LNS Position Quality UUID */
#define BT_UUID_LNS_POS_QUAL_VAL    0x2A69U

#define BT_UUID_LNS             BT_UUID_DECLARE_16(BT_UUID_LNS_VAL)
#define BT_UUID_LNS_LOC_SPEED   BT_UUID_DECLARE_16(BT_UUID_LNS_LOC_SPEED_VAL)
#define BT_UUID_LNS_POS_QUAL    BT_UUID_DECLARE_16(BT_UUID_LNS_POS_QUAL_VAL)

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Location notifications enabled */
static volatile bool loc_notifications_enabled;

/** Position quality notifications enabled */
static volatile bool qual_notifications_enabled;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * GATT Callbacks
 * ========================================================================== */

static void loc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    loc_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("LNS Location notifications %s",
            loc_notifications_enabled ? "enabled" : "disabled");
}

static void qual_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    qual_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("LNS Quality notifications %s",
            qual_notifications_enabled ? "enabled" : "disabled");
}

/* ==========================================================================
 * GATT Service Definition
 * ========================================================================== */

BT_GATT_SERVICE_DEFINE(lns_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_LNS),
    BT_GATT_CHARACTERISTIC(BT_UUID_LNS_LOC_SPEED,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(loc_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_LNS_POS_QUAL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          NULL, NULL, NULL),
    BT_GATT_CCC(qual_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ble_lns_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    loc_notifications_enabled = false;
    qual_notifications_enabled = false;
    is_initialized = true;

    LOG_INF("LNS initialized");
    return APP_OK;
}

app_err_t ble_lns_update_location(const loc_data_t *loc)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (loc == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    if (!loc_notifications_enabled) {
        return APP_OK;  /* No subscribers */
    }

    /*
     * Location and Speed characteristic format:
     * - Flags (2 bytes)
     * - Instantaneous Speed (2 bytes, 0.01 m/s)
     * - Total Distance (3 bytes, meters)
     * - Latitude (4 bytes, 1e-7 degrees)
     * - Longitude (4 bytes, 1e-7 degrees)
     * - Elevation (3 bytes, 0.01 meters)
     * - Heading (2 bytes, 0.01 degrees)
     */

    uint8_t data[20];
    uint16_t offset = 0U;

    /* Flags: Speed, Location, Elevation, Heading present */
    uint16_t flags = LNS_FLAG_SPEED_PRESENT |
                     LNS_FLAG_LOCATION_PRESENT |
                     LNS_FLAG_ELEVATION_PRESENT |
                     LNS_FLAG_HEADING_PRESENT;

    data[offset++] = (uint8_t)(flags & 0xFFU);
    data[offset++] = (uint8_t)((flags >> 8) & 0xFFU);

    /* Speed in 0.01 m/s */
    uint16_t speed_cms = (uint16_t)((loc->speed / 3.6f) * 100.0f);
    data[offset++] = (uint8_t)(speed_cms & 0xFFU);
    data[offset++] = (uint8_t)((speed_cms >> 8) & 0xFFU);

    /* Latitude in 1e-7 degrees */
    int32_t lat_int = (int32_t)(loc->lat * 10000000.0f);
    data[offset++] = (uint8_t)(lat_int & 0xFFU);
    data[offset++] = (uint8_t)((lat_int >> 8) & 0xFFU);
    data[offset++] = (uint8_t)((lat_int >> 16) & 0xFFU);
    data[offset++] = (uint8_t)((lat_int >> 24) & 0xFFU);

    /* Longitude in 1e-7 degrees */
    int32_t lon_int = (int32_t)(loc->lon * 10000000.0f);
    data[offset++] = (uint8_t)(lon_int & 0xFFU);
    data[offset++] = (uint8_t)((lon_int >> 8) & 0xFFU);
    data[offset++] = (uint8_t)((lon_int >> 16) & 0xFFU);
    data[offset++] = (uint8_t)((lon_int >> 24) & 0xFFU);

    /* Elevation in 0.01 meters (24-bit signed) */
    int32_t elev_cm = (int32_t)(loc->alt * 100.0f);
    data[offset++] = (uint8_t)(elev_cm & 0xFFU);
    data[offset++] = (uint8_t)((elev_cm >> 8) & 0xFFU);
    data[offset++] = (uint8_t)((elev_cm >> 16) & 0xFFU);

    /* Heading in 0.01 degrees */
    uint16_t heading = (uint16_t)(loc->course * 100.0f);
    data[offset++] = (uint8_t)(heading & 0xFFU);
    data[offset++] = (uint8_t)((heading >> 8) & 0xFFU);

    /* Get attribute and notify */
    const struct bt_gatt_attr *attr = &lns_svc.attrs[2];

    int err = bt_gatt_notify(NULL, attr, data, offset);
    if (err < 0) {
        LOG_ERR("LNS location notify failed: %d", err);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t ble_lns_update_quality(float hdop, float vdop, uint8_t satellites)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (!qual_notifications_enabled) {
        return APP_OK;
    }

    /*
     * Position Quality characteristic format:
     * - Flags (2 bytes)
     * - Number of Beacons in Solution (1 byte)
     * - HDOP (1 byte, 0.2 resolution)
     * - VDOP (1 byte, 0.2 resolution)
     */

    uint8_t data[5];
    uint16_t flags = 0x0007U;  /* Beacons, HDOP, VDOP present */

    data[0] = (uint8_t)(flags & 0xFFU);
    data[1] = (uint8_t)((flags >> 8) & 0xFFU);
    data[2] = satellites;
    data[3] = (uint8_t)(hdop * 5.0f);   /* 0.2 resolution */
    data[4] = (uint8_t)(vdop * 5.0f);

    const struct bt_gatt_attr *attr = &lns_svc.attrs[5];

    int err = bt_gatt_notify(NULL, attr, data, sizeof(data));
    if (err < 0) {
        LOG_ERR("LNS quality notify failed: %d", err);
        return APP_ERR_IO;
    }

    return APP_OK;
}

bool ble_lns_notifications_enabled(void)
{
    return loc_notifications_enabled;
}
