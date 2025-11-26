/**
 * @file neopixel.h
 * @brief NeoPixel (WS2812B) LED driver for Zephyr
 *
 * Uses PWM with DMA for timing-critical WS2812B protocol.
 */

#ifndef DRIVERS_NEOPIXEL_H
#define DRIVERS_NEOPIXEL_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum number of LEDs supported */
#define NEOPIXEL_MAX_LEDS       8U

/* ==========================================================================
 * Predefined Colors (R, G, B)
 * ========================================================================== */

#define NEOPIXEL_BLACK      0x00, 0x00, 0x00
#define NEOPIXEL_RED        0xFF, 0x00, 0x00
#define NEOPIXEL_GREEN      0x00, 0xFF, 0x00
#define NEOPIXEL_BLUE       0x00, 0x00, 0xFF
#define NEOPIXEL_YELLOW     0xFF, 0xFF, 0x00
#define NEOPIXEL_ORANGE     0xFF, 0x99, 0x33
#define NEOPIXEL_PURPLE     0xFF, 0x33, 0xFF
#define NEOPIXEL_WHITE      0xFF, 0xFF, 0xFF
#define NEOPIXEL_CYAN       0x00, 0xFF, 0xFF
#define NEOPIXEL_MAGENTA    0xFF, 0x00, 0xFF

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** RGB color structure */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} neopixel_color_t;

/** Neopixel strip handle */
typedef struct {
    neopixel_color_t leds[NEOPIXEL_MAX_LEDS];
    uint16_t num_leds;
    uint8_t brightness;     /**< Global brightness (0-255) */
    bool initialized;
} neopixel_strip_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize NeoPixel strip
 * @param strip Pointer to strip structure
 * @param num_leds Number of LEDs (up to NEOPIXEL_MAX_LEDS)
 * @return APP_OK on success
 */
app_err_t neopixel_init(neopixel_strip_t *strip, uint16_t num_leds);

/**
 * @brief Deinitialize NeoPixel strip
 * @param strip Pointer to strip structure
 */
void neopixel_deinit(neopixel_strip_t *strip);

/**
 * @brief Clear all LEDs (set to black)
 * @param strip Pointer to strip structure
 */
void neopixel_clear(neopixel_strip_t *strip);

/**
 * @brief Update strip with current LED data
 * @param strip Pointer to strip structure
 * @return APP_OK on success
 */
app_err_t neopixel_show(neopixel_strip_t *strip);

/**
 * @brief Set color of a single LED
 * @param strip Pointer to strip structure
 * @param index LED index (0-based)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @return APP_OK on success, APP_ERR_INVALID_PARAM if index out of range
 */
app_err_t neopixel_set_color(neopixel_strip_t *strip, uint16_t index,
                              uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set color and immediately update strip
 * @param strip Pointer to strip structure
 * @param index LED index (0-based)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @return APP_OK on success
 */
app_err_t neopixel_set_color_and_show(neopixel_strip_t *strip, uint16_t index,
                                       uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set all LEDs to the same color
 * @param strip Pointer to strip structure
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void neopixel_fill(neopixel_strip_t *strip, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set global brightness
 * @param strip Pointer to strip structure
 * @param brightness Brightness value (0-255)
 */
void neopixel_set_brightness(neopixel_strip_t *strip, uint8_t brightness);

/**
 * @brief Get color of a LED
 * @param strip Pointer to strip structure
 * @param index LED index
 * @return Pointer to color structure or NULL if invalid
 */
const neopixel_color_t *neopixel_get_color(const neopixel_strip_t *strip, uint16_t index);

/**
 * @brief Create color from HSV values
 * @param h Hue (0-360)
 * @param s Saturation (0-100)
 * @param v Value/Brightness (0-100)
 * @return RGB color
 */
neopixel_color_t neopixel_hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v);

/**
 * @brief Rainbow effect - set each LED to a different hue
 * @param strip Pointer to strip structure
 * @param start_hue Starting hue (0-360)
 */
void neopixel_rainbow(neopixel_strip_t *strip, uint16_t start_hue);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_NEOPIXEL_H */
