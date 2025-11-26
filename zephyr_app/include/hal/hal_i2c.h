/**
 * @file hal_i2c.h
 * @brief I2C Hardware Abstraction Layer for stravaV10
 *
 * Provides abstracted I2C operations for sensors.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>
#include <stddef.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** I2C device addresses */
#define HAL_I2C_ADDR_BARO       0x76U   /**< BMP280/BME280 barometer */
#define HAL_I2C_ADDR_FXOS       0x1EU   /**< FXOS8700 accelerometer */
#define HAL_I2C_ADDR_STC3100    0x70U   /**< STC3100 fuel gauge */

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** I2C bus identifiers */
typedef enum {
    HAL_I2C_SENSORS = 0,    /**< I2C bus for sensors (I2C0) */
    HAL_I2C_BUS_COUNT
} hal_i2c_bus_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize I2C subsystem
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_init(void);

/**
 * @brief Write data to I2C device
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @param data Data buffer to write
 * @param len Number of bytes to write
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_write(hal_i2c_bus_t bus, uint8_t addr,
                        const uint8_t *data, size_t len);

/**
 * @brief Read data from I2C device
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_read(hal_i2c_bus_t bus, uint8_t addr,
                       uint8_t *data, size_t len);

/**
 * @brief Write then read from I2C device (combined transaction)
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @param tx_data Data buffer to write
 * @param tx_len Number of bytes to write
 * @param rx_data Buffer to store read data
 * @param rx_len Number of bytes to read
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_write_read(hal_i2c_bus_t bus, uint8_t addr,
                             const uint8_t *tx_data, size_t tx_len,
                             uint8_t *rx_data, size_t rx_len);

/**
 * @brief Write to a register on I2C device
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @param reg Register address
 * @param value Value to write
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_write_reg(hal_i2c_bus_t bus, uint8_t addr,
                            uint8_t reg, uint8_t value);

/**
 * @brief Read from a register on I2C device
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @param reg Register address
 * @param value Pointer to store read value
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_read_reg(hal_i2c_bus_t bus, uint8_t addr,
                           uint8_t reg, uint8_t *value);

/**
 * @brief Read multiple bytes from a register on I2C device
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @param reg Starting register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_i2c_read_regs(hal_i2c_bus_t bus, uint8_t addr,
                            uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Check if I2C device is present
 * @param bus I2C bus identifier
 * @param addr Device address (7-bit)
 * @return true if device responds, false otherwise
 */
bool hal_i2c_device_ready(hal_i2c_bus_t bus, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* HAL_I2C_H */
