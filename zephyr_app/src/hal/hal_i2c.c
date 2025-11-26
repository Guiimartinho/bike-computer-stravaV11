/**
 * @file hal_i2c.c
 * @brief I2C Hardware Abstraction Layer implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "hal/hal_i2c.h"

LOG_MODULE_REGISTER(hal_i2c, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

/* I2C0 - Sensors */
static const struct device *i2c_sensors_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** I2C devices per bus */
static const struct device *i2c_devs[HAL_I2C_BUS_COUNT] = {
    [HAL_I2C_SENSORS] = NULL,
};

/** Initialization flag */
static bool is_initialized;

/** Mutex for thread safety */
static struct k_mutex i2c_mutex;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t hal_i2c_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Check I2C sensors device */
    if (!device_is_ready(i2c_sensors_dev)) {
        LOG_ERR("I2C sensors device not ready");
        return APP_ERR_NOT_INIT;
    }
    i2c_devs[HAL_I2C_SENSORS] = i2c_sensors_dev;

    /* Initialize mutex */
    k_mutex_init(&i2c_mutex);

    is_initialized = true;
    LOG_INF("I2C HAL initialized");

    return APP_OK;
}

app_err_t hal_i2c_write(hal_i2c_bus_t bus, uint8_t addr,
                        const uint8_t *data, size_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_I2C_BUS_COUNT) || (data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    if (k_mutex_lock(&i2c_mutex, K_MSEC(100)) != 0) {
        return APP_ERR_TIMEOUT;
    }

    int ret = i2c_write(i2c_devs[bus], data, len, addr);

    k_mutex_unlock(&i2c_mutex);

    if (ret < 0) {
        LOG_ERR("I2C write failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_i2c_read(hal_i2c_bus_t bus, uint8_t addr,
                       uint8_t *data, size_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_I2C_BUS_COUNT) || (data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    if (k_mutex_lock(&i2c_mutex, K_MSEC(100)) != 0) {
        return APP_ERR_TIMEOUT;
    }

    int ret = i2c_read(i2c_devs[bus], data, len, addr);

    k_mutex_unlock(&i2c_mutex);

    if (ret < 0) {
        LOG_ERR("I2C read failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_i2c_write_read(hal_i2c_bus_t bus, uint8_t addr,
                             const uint8_t *tx_data, size_t tx_len,
                             uint8_t *rx_data, size_t rx_len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_I2C_BUS_COUNT) || (tx_data == NULL) || (tx_len == 0U) ||
        (rx_data == NULL) || (rx_len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    if (k_mutex_lock(&i2c_mutex, K_MSEC(100)) != 0) {
        return APP_ERR_TIMEOUT;
    }

    int ret = i2c_write_read(i2c_devs[bus], addr, tx_data, tx_len, rx_data, rx_len);

    k_mutex_unlock(&i2c_mutex);

    if (ret < 0) {
        LOG_ERR("I2C write_read failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_i2c_write_reg(hal_i2c_bus_t bus, uint8_t addr,
                            uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return hal_i2c_write(bus, addr, buf, sizeof(buf));
}

app_err_t hal_i2c_read_reg(hal_i2c_bus_t bus, uint8_t addr,
                           uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    return hal_i2c_write_read(bus, addr, &reg, 1U, value, 1U);
}

app_err_t hal_i2c_read_regs(hal_i2c_bus_t bus, uint8_t addr,
                            uint8_t reg, uint8_t *data, size_t len)
{
    return hal_i2c_write_read(bus, addr, &reg, 1U, data, len);
}

bool hal_i2c_device_ready(hal_i2c_bus_t bus, uint8_t addr)
{
    if (!is_initialized || (bus >= HAL_I2C_BUS_COUNT)) {
        return false;
    }

    if (k_mutex_lock(&i2c_mutex, K_MSEC(100)) != 0) {
        return false;
    }

    /* Send empty message to check if device responds */
    struct i2c_msg msg = {
        .buf = NULL,
        .len = 0U,
        .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
    };

    int ret = i2c_transfer(i2c_devs[bus], &msg, 1U, addr);

    k_mutex_unlock(&i2c_mutex);

    return (ret == 0);
}
