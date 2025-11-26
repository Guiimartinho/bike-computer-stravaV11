/**
 * @file stc3100.c
 * @brief STC3100 Battery Fuel Gauge driver implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "drivers/stc3100.h"
#include "hal/hal_i2c.h"

LOG_MODULE_REGISTER(stc3100, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** STC3100 I2C address */
#define STC3100_ADDR            0x70U

/** Register addresses */
#define REG_MODE                0x00U
#define REG_CONTROL             0x01U
#define REG_CHARGE_LOW          0x02U
#define REG_CHARGE_HIGH         0x03U
#define REG_COUNTER_LOW         0x04U
#define REG_COUNTER_HIGH        0x05U
#define REG_CURRENT_LOW         0x06U
#define REG_CURRENT_HIGH        0x07U
#define REG_VOLTAGE_LOW         0x08U
#define REG_VOLTAGE_HIGH        0x09U
#define REG_TEMP_LOW            0x0AU
#define REG_TEMP_HIGH           0x0BU
#define REG_DEVICE_ID           0x18U

/** Mode register bits */
#define MODE_RUN                0x10U
#define MODE_GG_RUN             0x10U

/** Control register bits */
#define CTRL_IO_OD              0x01U
#define CTRL_GG_RST             0x02U
#define CTRL_GG_VM              0x04U
#define CTRL_PORDET             0x10U

/** Conversion constants */
#define VOLTAGE_LSB_UV          2440U       /**< 2.44 mV per LSB */
#define CURRENT_LSB_FACTOR      11770U      /**< 11.77 uV / Rsense per LSB */
#define TEMP_LSB_C              125U        /**< 0.125 C per LSB (8 LSBs per degree) */
#define CHARGE_LSB_FACTOR       6700U       /**< 6.7 uVh / Rsense per LSB */

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Battery configuration */
static battery_config_t batt_config = {
    .rsense = 100U,         /**< Default 100 mOhm sense resistor */
    .capacity_mah = 1000U,  /**< Default 1000 mAh battery */
    .voltage_min = 3000U,   /**< 3.0V minimum */
    .voltage_max = 4200U,   /**< 4.2V maximum */
};

/** Latest battery data */
static battery_data_t latest_data;

/** Charge offset for reset */
static float charge_offset;

/** Average current calculation */
static struct {
    float sum;
    uint32_t count;
    uint32_t start_time;
} avg_current;

/** Data ready flag */
static volatile bool data_ready;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Read register value
 */
static app_err_t read_reg(uint8_t reg, uint8_t *value)
{
    return hal_i2c_read_reg(HAL_I2C_SENSORS, STC3100_ADDR, reg, value);
}

/**
 * @brief Write register value
 */
static app_err_t write_reg(uint8_t reg, uint8_t value)
{
    return hal_i2c_write_reg(HAL_I2C_SENSORS, STC3100_ADDR, reg, value);
}

/**
 * @brief Read multiple registers
 */
static app_err_t read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    return hal_i2c_read_regs(HAL_I2C_SENSORS, STC3100_ADDR, start_reg, data, len);
}

/**
 * @brief Calculate state of charge from voltage
 */
static uint8_t voltage_to_soc(float voltage_mv)
{
    if (voltage_mv <= (float)batt_config.voltage_min) {
        return 0U;
    }

    if (voltage_mv >= (float)batt_config.voltage_max) {
        return 100U;
    }

    float range = (float)(batt_config.voltage_max - batt_config.voltage_min);
    float offset = voltage_mv - (float)batt_config.voltage_min;

    return (uint8_t)((offset / range) * 100.0f);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t stc3100_init(const battery_config_t *config)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    if (config != NULL) {
        batt_config = *config;
    }

    /* Check device presence */
    if (!hal_i2c_device_ready(HAL_I2C_SENSORS, STC3100_ADDR)) {
        LOG_ERR("STC3100 not found at address 0x%02X", STC3100_ADDR);
        return APP_ERR_NOT_FOUND;
    }

    /* Read device ID */
    uint8_t dev_id = 0U;
    app_err_t err = read_reg(REG_DEVICE_ID, &dev_id);
    if (err != APP_OK) {
        LOG_ERR("Failed to read device ID");
        return err;
    }

    LOG_INF("STC3100 device ID: 0x%02X", dev_id);

    /* Reset the gas gauge */
    err = write_reg(REG_CONTROL, CTRL_GG_RST);
    if (err != APP_OK) {
        LOG_ERR("Failed to reset STC3100");
        return err;
    }

    k_msleep(10);

    /* Start the gas gauge in run mode */
    err = write_reg(REG_MODE, MODE_RUN);
    if (err != APP_OK) {
        LOG_ERR("Failed to start STC3100");
        return err;
    }

    /* Clear data */
    (void)memset(&latest_data, 0, sizeof(latest_data));
    (void)memset(&avg_current, 0, sizeof(avg_current));
    charge_offset = 0.0f;

    is_initialized = true;
    LOG_INF("STC3100 initialized (Rsense=%u mOhm, Capacity=%u mAh)",
            batt_config.rsense, batt_config.capacity_mah);

    return APP_OK;
}

app_err_t stc3100_trigger(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Read all data registers at once */
    uint8_t raw_data[10];
    app_err_t err = read_regs(REG_CHARGE_LOW, raw_data, sizeof(raw_data));
    if (err != APP_OK) {
        LOG_ERR("Failed to read STC3100 data");
        return err;
    }

    /* Parse charge (registers 0-1 in buffer) */
    int16_t charge_raw = (int16_t)((uint16_t)raw_data[1] << 8) | raw_data[0];
    /* Charge LSB = 6.7 uVh / Rsense */
    float charge_mah = (float)charge_raw * (float)CHARGE_LSB_FACTOR /
                       (float)batt_config.rsense / 1000.0f;
    latest_data.charge = charge_mah - charge_offset;

    /* Parse counter (registers 2-3 in buffer) */
    /* uint16_t counter = ((uint16_t)raw_data[3] << 8) | raw_data[2]; */

    /* Parse current (registers 4-5 in buffer) */
    int16_t current_raw = (int16_t)((uint16_t)raw_data[5] << 8) | raw_data[4];
    /* Current LSB = 11.77 uV / Rsense */
    latest_data.current = (float)current_raw * (float)CURRENT_LSB_FACTOR /
                          (float)batt_config.rsense / 1000.0f;

    /* Parse voltage (registers 6-7 in buffer) */
    uint16_t voltage_raw = ((uint16_t)raw_data[7] << 8) | raw_data[6];
    voltage_raw &= 0x0FFFU;  /* 12-bit value */
    latest_data.voltage = (float)voltage_raw * (float)VOLTAGE_LSB_UV / 1000.0f;

    /* Parse temperature (registers 8-9 in buffer) */
    int16_t temp_raw = (int16_t)((uint16_t)raw_data[9] << 8) | raw_data[8];
    latest_data.temperature = (float)temp_raw / 8.0f;  /* 8 LSBs per degree C */

    /* Calculate state of charge */
    latest_data.soc = voltage_to_soc(latest_data.voltage);

    /* Update average current */
    avg_current.sum += latest_data.current;
    avg_current.count++;

    /* Update timestamp and validity */
    latest_data.timestamp = k_uptime_get_32();
    latest_data.valid = true;
    data_ready = true;

    return APP_OK;
}

app_err_t stc3100_read(battery_data_t *data)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (data == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    *data = latest_data;
    data_ready = false;

    return APP_OK;
}

float stc3100_get_voltage(void)
{
    return latest_data.voltage;
}

float stc3100_get_current(void)
{
    return latest_data.current;
}

float stc3100_get_temperature(void)
{
    return latest_data.temperature;
}

uint8_t stc3100_get_soc(void)
{
    return latest_data.soc;
}

float stc3100_get_charge(void)
{
    return latest_data.charge;
}

void stc3100_reset_charge(void)
{
    charge_offset = latest_data.charge + charge_offset;
    latest_data.charge = 0.0f;

    /* Reset average current calculation */
    avg_current.sum = 0.0f;
    avg_current.count = 0U;
    avg_current.start_time = k_uptime_get_32();
}

float stc3100_get_avg_current(void)
{
    if (avg_current.count == 0U) {
        return 0.0f;
    }

    return avg_current.sum / (float)avg_current.count;
}

app_err_t stc3100_sleep(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Stop the gas gauge */
    return write_reg(REG_MODE, 0x00U);
}

app_err_t stc3100_wake(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    /* Start the gas gauge */
    return write_reg(REG_MODE, MODE_RUN);
}

app_err_t stc3100_reset(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    app_err_t err = write_reg(REG_CONTROL, CTRL_GG_RST);
    if (err != APP_OK) {
        return err;
    }

    k_msleep(10);

    /* Restart gas gauge */
    err = write_reg(REG_MODE, MODE_RUN);
    if (err != APP_OK) {
        return err;
    }

    /* Reset state */
    charge_offset = 0.0f;
    (void)memset(&avg_current, 0, sizeof(avg_current));

    return APP_OK;
}

bool stc3100_is_data_ready(void)
{
    return data_ready;
}
