/**
 * @file hal_spi.h
 * @brief SPI Hardware Abstraction Layer for stravaV10
 *
 * Provides abstracted SPI operations for LCD and SD card.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>
#include <stddef.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** SPI bus identifiers */
typedef enum {
    HAL_SPI_LCD = 0,    /**< SPI bus for LCD (SPI1) */
    HAL_SPI_SDC,        /**< SPI bus for SD Card (SPI2) */
    HAL_SPI_BUS_COUNT
} hal_spi_bus_t;

/** SPI configuration structure */
typedef struct {
    uint32_t frequency;     /**< SPI clock frequency in Hz */
    uint8_t mode;           /**< SPI mode (0-3) */
    bool lsb_first;         /**< LSB first transmission */
} hal_spi_config_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize SPI subsystem
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_spi_init(void);

/**
 * @brief Configure SPI bus
 * @param bus SPI bus identifier
 * @param config Configuration parameters
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_spi_configure(hal_spi_bus_t bus, const hal_spi_config_t *config);

/**
 * @brief Transmit data over SPI
 * @param bus SPI bus identifier
 * @param data Data buffer to transmit
 * @param len Number of bytes to transmit
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_spi_transmit(hal_spi_bus_t bus, const uint8_t *data, size_t len);

/**
 * @brief Receive data over SPI
 * @param bus SPI bus identifier
 * @param data Buffer to store received data
 * @param len Number of bytes to receive
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_spi_receive(hal_spi_bus_t bus, uint8_t *data, size_t len);

/**
 * @brief Transmit and receive data over SPI
 * @param bus SPI bus identifier
 * @param tx_data Data buffer to transmit
 * @param rx_data Buffer to store received data
 * @param len Number of bytes to transfer
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_spi_transfer(hal_spi_bus_t bus, const uint8_t *tx_data,
                           uint8_t *rx_data, size_t len);

/**
 * @brief Assert chip select (active low)
 * @param bus SPI bus identifier
 */
void hal_spi_cs_assert(hal_spi_bus_t bus);

/**
 * @brief Deassert chip select
 * @param bus SPI bus identifier
 */
void hal_spi_cs_deassert(hal_spi_bus_t bus);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H */
