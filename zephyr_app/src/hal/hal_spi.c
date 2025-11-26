/**
 * @file hal_spi.c
 * @brief SPI Hardware Abstraction Layer implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "hal/hal_spi.h"

LOG_MODULE_REGISTER(hal_spi, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

/* SPI1 - LCD */
static const struct device *spi_lcd_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec lcd_cs = GPIO_DT_SPEC_GET(DT_NODELABEL(spi1), cs_gpios);

/* SPI2 - SD Card */
static const struct device *spi_sdc_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
static const struct gpio_dt_spec sdc_cs = GPIO_DT_SPEC_GET(DT_NODELABEL(spi2), cs_gpios);

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** SPI configurations per bus */
static struct spi_config spi_configs[HAL_SPI_BUS_COUNT];

/** CS GPIO specs per bus */
static const struct gpio_dt_spec *cs_gpios[HAL_SPI_BUS_COUNT] = {
    [HAL_SPI_LCD] = &lcd_cs,
    [HAL_SPI_SDC] = &sdc_cs,
};

/** SPI devices per bus */
static const struct device *spi_devs[HAL_SPI_BUS_COUNT] = {
    [HAL_SPI_LCD] = NULL,
    [HAL_SPI_SDC] = NULL,
};

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t hal_spi_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Check SPI LCD device */
    if (!device_is_ready(spi_lcd_dev)) {
        LOG_ERR("SPI LCD device not ready");
        return APP_ERR_NOT_INIT;
    }
    spi_devs[HAL_SPI_LCD] = spi_lcd_dev;

    /* Check SPI SDC device */
    if (!device_is_ready(spi_sdc_dev)) {
        LOG_ERR("SPI SDC device not ready");
        return APP_ERR_NOT_INIT;
    }
    spi_devs[HAL_SPI_SDC] = spi_sdc_dev;

    /* Configure CS pins */
    if (gpio_is_ready_dt(&lcd_cs)) {
        int ret = gpio_pin_configure_dt(&lcd_cs, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure LCD CS: %d", ret);
            return APP_ERR_IO;
        }
    }

    if (gpio_is_ready_dt(&sdc_cs)) {
        int ret = gpio_pin_configure_dt(&sdc_cs, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure SDC CS: %d", ret);
            return APP_ERR_IO;
        }
    }

    /* Default LCD SPI configuration */
    spi_configs[HAL_SPI_LCD].frequency = 2000000U;  /* 2 MHz for LS027 */
    spi_configs[HAL_SPI_LCD].operation = SPI_WORD_SET(8) |
                                         SPI_TRANSFER_LSB |
                                         SPI_OP_MODE_MASTER;
    spi_configs[HAL_SPI_LCD].slave = 0U;
    spi_configs[HAL_SPI_LCD].cs.gpio = lcd_cs;
    spi_configs[HAL_SPI_LCD].cs.delay = 0U;

    /* Default SD Card SPI configuration */
    spi_configs[HAL_SPI_SDC].frequency = 8000000U;  /* 8 MHz for SD */
    spi_configs[HAL_SPI_SDC].operation = SPI_WORD_SET(8) |
                                         SPI_OP_MODE_MASTER;
    spi_configs[HAL_SPI_SDC].slave = 0U;
    spi_configs[HAL_SPI_SDC].cs.gpio = sdc_cs;
    spi_configs[HAL_SPI_SDC].cs.delay = 0U;

    is_initialized = true;
    LOG_INF("SPI HAL initialized");

    return APP_OK;
}

app_err_t hal_spi_configure(hal_spi_bus_t bus, const hal_spi_config_t *config)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_SPI_BUS_COUNT) || (config == NULL)) {
        return APP_ERR_INVALID_PARAM;
    }

    spi_configs[bus].frequency = config->frequency;

    uint16_t operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;

    if (config->lsb_first) {
        operation |= SPI_TRANSFER_LSB;
    }

    switch (config->mode) {
    case 0U:
        /* CPOL=0, CPHA=0 (default) */
        break;
    case 1U:
        operation |= SPI_MODE_CPHA;
        break;
    case 2U:
        operation |= SPI_MODE_CPOL;
        break;
    case 3U:
        operation |= SPI_MODE_CPOL | SPI_MODE_CPHA;
        break;
    default:
        return APP_ERR_INVALID_PARAM;
    }

    spi_configs[bus].operation = operation;

    return APP_OK;
}

app_err_t hal_spi_transmit(hal_spi_bus_t bus, const uint8_t *data, size_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_SPI_BUS_COUNT) || (data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    struct spi_buf tx_buf = {
        .buf = (void *)data,
        .len = len,
    };

    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1U,
    };

    int ret = spi_write(spi_devs[bus], &spi_configs[bus], &tx_bufs);
    if (ret < 0) {
        LOG_ERR("SPI transmit failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_spi_receive(hal_spi_bus_t bus, uint8_t *data, size_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_SPI_BUS_COUNT) || (data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    struct spi_buf rx_buf = {
        .buf = data,
        .len = len,
    };

    struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1U,
    };

    int ret = spi_read(spi_devs[bus], &spi_configs[bus], &rx_bufs);
    if (ret < 0) {
        LOG_ERR("SPI receive failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_spi_transfer(hal_spi_bus_t bus, const uint8_t *tx_data,
                           uint8_t *rx_data, size_t len)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if ((bus >= HAL_SPI_BUS_COUNT) || (tx_data == NULL) ||
        (rx_data == NULL) || (len == 0U)) {
        return APP_ERR_INVALID_PARAM;
    }

    struct spi_buf tx_buf = {
        .buf = (void *)tx_data,
        .len = len,
    };

    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1U,
    };

    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = len,
    };

    struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1U,
    };

    int ret = spi_transceive(spi_devs[bus], &spi_configs[bus], &tx_bufs, &rx_bufs);
    if (ret < 0) {
        LOG_ERR("SPI transfer failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

void hal_spi_cs_assert(hal_spi_bus_t bus)
{
    if ((bus >= HAL_SPI_BUS_COUNT) || (cs_gpios[bus] == NULL)) {
        return;
    }

    /* LCD CS is active high, SDC CS is active low */
    if (bus == HAL_SPI_LCD) {
        (void)gpio_pin_set_dt(cs_gpios[bus], 1);
    } else {
        (void)gpio_pin_set_dt(cs_gpios[bus], 0);
    }
}

void hal_spi_cs_deassert(hal_spi_bus_t bus)
{
    if ((bus >= HAL_SPI_BUS_COUNT) || (cs_gpios[bus] == NULL)) {
        return;
    }

    /* LCD CS is active high, SDC CS is active low */
    if (bus == HAL_SPI_LCD) {
        (void)gpio_pin_set_dt(cs_gpios[bus], 0);
    } else {
        (void)gpio_pin_set_dt(cs_gpios[bus], 1);
    }
}
