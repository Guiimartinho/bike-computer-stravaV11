/**
 * @file ls027.c
 * @brief Sharp Memory LCD LS027B4DH01 driver implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "drivers/ls027.h"

LOG_MODULE_REGISTER(ls027, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** SPI buffer size including command bytes */
#define SPI_BUF_SIZE    (1U + LS027_BUFFER_SIZE + (LS027_HEIGHT * 2U) + 1U)

/** LCD command bits */
#define CMD_WRITE       0x01U   /**< Write line command */
#define CMD_VCOM        0x02U   /**< VCOM bit */
#define CMD_CLEAR       0x04U   /**< Clear screen command */

/** Bit manipulation helpers */
static const uint8_t bit_set[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
static const uint8_t bit_clr[8] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(spi1), cs_gpios);

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** SPI framebuffer with command bytes */
static uint8_t spi_buffer[SPI_BUF_SIZE];

/** Current VCOM state */
static uint8_t vcom_bit;

/** Color inversion flag */
static bool color_inverted = true;

/** Display orientation */
static ls027_orient_t orientation = LS027_ORIENT_LANDSCAPE;

/** Busy flag */
static volatile bool is_busy;

/** Initialization flag */
static bool is_initialized;

/** Mutex for thread safety */
static struct k_mutex lcd_mutex;

/** SPI configuration */
static struct spi_config spi_cfg = {
    .frequency = 2000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_LSB | SPI_OP_MODE_MASTER,
    .slave = 0U,
};

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate buffer index for pixel
 */
static inline uint32_t pixel_to_index(uint16_t x, uint16_t y)
{
    /* Buffer format: CMD + (LINE_ADDR + 50 bytes data + DUMMY) * 240 + DUMMY */
    /* Each line: 1 byte addr + 50 bytes data + 1 byte dummy = 52 bytes */
    return 2U + ((uint32_t)y * LS027_WIDTH + x) / 8U + (2U * y);
}

/**
 * @brief Prepare SPI buffer with line addresses
 */
static void prepare_spi_buffer(void)
{
    uint32_t addr = 0U;

    /* Write command with VCOM bit */
    spi_buffer[addr] = CMD_WRITE | vcom_bit;
    addr++;

    /* Toggle VCOM for next transfer */
    vcom_bit = (vcom_bit != 0U) ? 0U : CMD_VCOM;

    /* Set line addresses and dummy bytes */
    for (uint16_t line = 0U; line < LS027_HEIGHT; line++) {
        /* Line address (1-indexed) */
        spi_buffer[addr] = (uint8_t)(line + 1U);
        addr++;

        /* Skip data bytes (already in buffer) */
        addr += LS027_WIDTH / 8U;

        /* Dummy byte after line data */
        spi_buffer[addr] = 0x00U;
        addr++;
    }

    /* Final dummy byte */
    spi_buffer[addr] = 0x00U;
}

/**
 * @brief Clear SPI buffer to default color
 */
static void clear_buffer_internal(void)
{
    uint8_t fill = color_inverted ? 0xFFU : 0x00U;

    for (uint16_t line = 0U; line < LS027_HEIGHT; line++) {
        uint32_t line_start = 2U + (line * (LS027_WIDTH / 8U + 2U));
        (void)memset(&spi_buffer[line_start], fill, LS027_WIDTH / 8U);
    }
}

/**
 * @brief Send SPI data to LCD
 */
static app_err_t spi_send(const uint8_t *data, size_t len)
{
    struct spi_buf tx_buf = {
        .buf = (void *)data,
        .len = len,
    };

    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1U,
    };

    /* Assert CS (active high for this LCD) */
    (void)gpio_pin_set_dt(&cs_gpio, 1);

    int ret = spi_write(spi_dev, &spi_cfg, &tx_bufs);

    /* Deassert CS */
    (void)gpio_pin_set_dt(&cs_gpio, 0);

    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

/**
 * @brief Transform coordinates based on orientation
 */
static void transform_coords(uint16_t *x, uint16_t *y)
{
    uint16_t temp;

    switch (orientation) {
    case LS027_ORIENT_LANDSCAPE:
        /* No transformation needed */
        break;

    case LS027_ORIENT_LANDSCAPE_180:
        *x = (LS027_WIDTH - 1U) - *x;
        *y = (LS027_HEIGHT - 1U) - *y;
        break;

    case LS027_ORIENT_PORTRAIT:
        temp = *x;
        *x = *y;
        *y = (LS027_WIDTH - 1U) - temp;
        break;

    case LS027_ORIENT_PORTRAIT_180:
        temp = *x;
        *x = (LS027_HEIGHT - 1U) - *y;
        *y = temp;
        break;

    default:
        /* No transformation */
        break;
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t ls027_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Check SPI device */
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return APP_ERR_NOT_INIT;
    }

    /* Configure CS GPIO */
    if (!gpio_is_ready_dt(&cs_gpio)) {
        LOG_ERR("CS GPIO not ready");
        return APP_ERR_NOT_INIT;
    }

    int ret = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure CS GPIO: %d", ret);
        return APP_ERR_IO;
    }

    /* Initialize mutex */
    k_mutex_init(&lcd_mutex);

    /* Initialize VCOM state */
    vcom_bit = CMD_VCOM;

    /* Clear buffer */
    (void)memset(spi_buffer, 0, sizeof(spi_buffer));
    clear_buffer_internal();

    /* Send hardware clear command */
    uint8_t clear_cmd[2] = {CMD_CLEAR | vcom_bit, 0x00U};
    vcom_bit = (vcom_bit != 0U) ? 0U : CMD_VCOM;

    app_err_t err = spi_send(clear_cmd, sizeof(clear_cmd));
    if (err != APP_OK) {
        return err;
    }

    is_initialized = true;
    LOG_INF("LS027 LCD initialized");

    return APP_OK;
}

void ls027_clear(void)
{
    if (!is_initialized) {
        return;
    }

    if (k_mutex_lock(&lcd_mutex, K_MSEC(100)) != 0) {
        return;
    }

    clear_buffer_internal();

    k_mutex_unlock(&lcd_mutex);
}

app_err_t ls027_update(void)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (k_mutex_lock(&lcd_mutex, K_MSEC(100)) != 0) {
        return APP_ERR_TIMEOUT;
    }

    is_busy = true;

    /* Prepare buffer with line addresses */
    prepare_spi_buffer();

    /* Send entire buffer */
    app_err_t err = spi_send(spi_buffer, sizeof(spi_buffer));

    /* Clear buffer for next frame */
    clear_buffer_internal();

    is_busy = false;

    k_mutex_unlock(&lcd_mutex);

    return err;
}

void ls027_toggle_vcom(void)
{
    if (!is_initialized) {
        return;
    }

    /* Send VCOM toggle command */
    uint8_t vcom_cmd[2] = {vcom_bit, 0x00U};
    vcom_bit = (vcom_bit != 0U) ? 0U : CMD_VCOM;

    (void)spi_send(vcom_cmd, sizeof(vcom_cmd));
}

void ls027_draw_pixel(uint16_t x, uint16_t y, uint8_t color)
{
    if (!is_initialized) {
        return;
    }

    /* Transform coordinates based on orientation */
    transform_coords(&x, &y);

    /* Bounds check */
    if ((x >= LS027_WIDTH) || (y >= LS027_HEIGHT)) {
        return;
    }

    uint32_t idx = pixel_to_index(x, y);
    uint8_t bit = bit_set[x & 7U];

    /* Handle color modes */
    if (color == LS027_COLOR_INVERT) {
        spi_buffer[idx] ^= bit;
    } else if ((color != 0U) != color_inverted) {
        spi_buffer[idx] |= bit;
    } else {
        spi_buffer[idx] &= bit_clr[x & 7U];
    }
}

void ls027_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint8_t color)
{
    if (!is_initialized || (len == 0U)) {
        return;
    }

    /* Transform start coordinates */
    uint16_t x_start = x;
    uint16_t y_line = y;
    transform_coords(&x_start, &y_line);

    /* Handle orientation-dependent line direction */
    if ((orientation == LS027_ORIENT_PORTRAIT) ||
        (orientation == LS027_ORIENT_PORTRAIT_180)) {
        /* Horizontal line becomes vertical after rotation */
        for (uint16_t i = 0U; i < len; i++) {
            ls027_draw_pixel(x + i, y, color);
        }
        return;
    }

    /* Bounds check */
    if (y_line >= LS027_HEIGHT) {
        return;
    }

    if (x_start >= LS027_WIDTH) {
        x_start = 0U;
    }

    uint16_t end = x_start + len;
    if (end > LS027_WIDTH) {
        end = LS027_WIDTH;
    }

    /* Draw pixels */
    for (uint16_t px = x_start; px < end; px++) {
        uint32_t idx = pixel_to_index(px, y_line);
        uint8_t bit = bit_set[px & 7U];

        if (color == LS027_COLOR_INVERT) {
            spi_buffer[idx] ^= bit;
        } else if ((color != 0U) != color_inverted) {
            spi_buffer[idx] |= bit;
        } else {
            spi_buffer[idx] &= bit_clr[px & 7U];
        }
    }
}

void ls027_draw_vline(uint16_t x, uint16_t y, uint16_t len, uint8_t color)
{
    if (!is_initialized || (len == 0U)) {
        return;
    }

    for (uint16_t i = 0U; i < len; i++) {
        ls027_draw_pixel(x, y + i, color);
    }
}

void ls027_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color)
{
    if (!is_initialized || (w == 0U) || (h == 0U)) {
        return;
    }

    for (uint16_t row = 0U; row < h; row++) {
        ls027_draw_hline(x, y + row, w, color);
    }
}

uint8_t *ls027_get_buffer(void)
{
    return spi_buffer;
}

void ls027_invert_colors(void)
{
    color_inverted = !color_inverted;
}

void ls027_set_orientation(ls027_orient_t orient)
{
    orientation = orient;
}

uint16_t ls027_get_width(void)
{
    if ((orientation == LS027_ORIENT_PORTRAIT) ||
        (orientation == LS027_ORIENT_PORTRAIT_180)) {
        return LS027_HEIGHT;
    }
    return LS027_WIDTH;
}

uint16_t ls027_get_height(void)
{
    if ((orientation == LS027_ORIENT_PORTRAIT) ||
        (orientation == LS027_ORIENT_PORTRAIT_180)) {
        return LS027_WIDTH;
    }
    return LS027_HEIGHT;
}

bool ls027_is_busy(void)
{
    return is_busy;
}
