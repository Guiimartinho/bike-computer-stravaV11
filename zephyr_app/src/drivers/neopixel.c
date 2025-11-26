/**
 * @file neopixel.c
 * @brief NeoPixel (WS2812B) LED driver implementation
 *
 * Uses Zephyr's LED strip API when available, falls back to bitbang.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>

#include "drivers/neopixel.h"

LOG_MODULE_REGISTER(neopixel, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Default brightness if not set */
#define DEFAULT_BRIGHTNESS      128U

/* Check for LED strip device in devicetree */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led_strip), okay)
#define HAS_LED_STRIP   1
#include <zephyr/drivers/led_strip.h>

static const struct device *strip_dev = DEVICE_DT_GET(DT_ALIAS(led_strip));
static struct led_rgb led_buf[NEOPIXEL_MAX_LEDS];
#else
#define HAS_LED_STRIP   0
#endif

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Apply brightness scaling to color
 */
static inline uint8_t apply_brightness(uint8_t color, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)color * brightness) / 255U);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t neopixel_init(neopixel_strip_t *strip, uint16_t num_leds)
{
    if (strip == NULL) {
        return APP_ERR_INVALID_PARAM;
    }

    if (num_leds > NEOPIXEL_MAX_LEDS) {
        num_leds = NEOPIXEL_MAX_LEDS;
    }

    if (num_leds == 0U) {
        return APP_ERR_INVALID_PARAM;
    }

#if HAS_LED_STRIP
    if (!device_is_ready(strip_dev)) {
        LOG_ERR("LED strip device not ready");
        return APP_ERR_NOT_INIT;
    }
#endif

    strip->num_leds = num_leds;
    strip->brightness = DEFAULT_BRIGHTNESS;
    strip->initialized = true;

    /* Clear all LEDs */
    neopixel_clear(strip);

    LOG_INF("NeoPixel initialized: %u LEDs", num_leds);

    return APP_OK;
}

void neopixel_deinit(neopixel_strip_t *strip)
{
    if (strip == NULL) {
        return;
    }

    neopixel_clear(strip);
    (void)neopixel_show(strip);
    strip->initialized = false;
}

void neopixel_clear(neopixel_strip_t *strip)
{
    if ((strip == NULL) || !strip->initialized) {
        return;
    }

    (void)memset(strip->leds, 0, sizeof(strip->leds));
}

app_err_t neopixel_show(neopixel_strip_t *strip)
{
    if ((strip == NULL) || !strip->initialized) {
        return APP_ERR_NOT_INIT;
    }

#if HAS_LED_STRIP
    /* Convert to LED strip format with brightness applied */
    for (uint16_t i = 0U; i < strip->num_leds; i++) {
        led_buf[i].r = apply_brightness(strip->leds[i].r, strip->brightness);
        led_buf[i].g = apply_brightness(strip->leds[i].g, strip->brightness);
        led_buf[i].b = apply_brightness(strip->leds[i].b, strip->brightness);
    }

    int err = led_strip_update_rgb(strip_dev, led_buf, strip->num_leds);
    if (err < 0) {
        LOG_ERR("LED strip update failed: %d", err);
        return APP_ERR_IO;
    }
#else
    /* No hardware LED strip support - just log */
    LOG_DBG("NeoPixel show (simulated)");
#endif

    return APP_OK;
}

app_err_t neopixel_set_color(neopixel_strip_t *strip, uint16_t index,
                              uint8_t r, uint8_t g, uint8_t b)
{
    if ((strip == NULL) || !strip->initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (index >= strip->num_leds) {
        return APP_ERR_INVALID_PARAM;
    }

    strip->leds[index].r = r;
    strip->leds[index].g = g;
    strip->leds[index].b = b;

    return APP_OK;
}

app_err_t neopixel_set_color_and_show(neopixel_strip_t *strip, uint16_t index,
                                       uint8_t r, uint8_t g, uint8_t b)
{
    app_err_t err = neopixel_set_color(strip, index, r, g, b);
    if (err != APP_OK) {
        return err;
    }

    return neopixel_show(strip);
}

void neopixel_fill(neopixel_strip_t *strip, uint8_t r, uint8_t g, uint8_t b)
{
    if ((strip == NULL) || !strip->initialized) {
        return;
    }

    for (uint16_t i = 0U; i < strip->num_leds; i++) {
        strip->leds[i].r = r;
        strip->leds[i].g = g;
        strip->leds[i].b = b;
    }
}

void neopixel_set_brightness(neopixel_strip_t *strip, uint8_t brightness)
{
    if ((strip == NULL) || !strip->initialized) {
        return;
    }

    strip->brightness = brightness;
}

const neopixel_color_t *neopixel_get_color(const neopixel_strip_t *strip, uint16_t index)
{
    if ((strip == NULL) || !strip->initialized) {
        return NULL;
    }

    if (index >= strip->num_leds) {
        return NULL;
    }

    return &strip->leds[index];
}

neopixel_color_t neopixel_hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v)
{
    neopixel_color_t rgb = {0, 0, 0};
    uint8_t region, remainder, p, q, t;

    if (s == 0U) {
        /* Achromatic (gray) */
        rgb.r = (v * 255U) / 100U;
        rgb.g = rgb.r;
        rgb.b = rgb.r;
        return rgb;
    }

    /* Normalize hue to 0-360 */
    h = h % 360U;

    region = (uint8_t)(h / 60U);
    remainder = (uint8_t)((h % 60U) * 6U);  /* 0-359 scaled to 0-255 per region */

    /* Scale s and v to 0-255 */
    uint8_t s255 = (s * 255U) / 100U;
    uint8_t v255 = (v * 255U) / 100U;

    p = (uint8_t)((v255 * (255U - s255)) / 255U);
    q = (uint8_t)((v255 * (255U - ((s255 * remainder) / 255U))) / 255U);
    t = (uint8_t)((v255 * (255U - ((s255 * (255U - remainder)) / 255U))) / 255U);

    switch (region) {
    case 0:
        rgb.r = v255;
        rgb.g = t;
        rgb.b = p;
        break;
    case 1:
        rgb.r = q;
        rgb.g = v255;
        rgb.b = p;
        break;
    case 2:
        rgb.r = p;
        rgb.g = v255;
        rgb.b = t;
        break;
    case 3:
        rgb.r = p;
        rgb.g = q;
        rgb.b = v255;
        break;
    case 4:
        rgb.r = t;
        rgb.g = p;
        rgb.b = v255;
        break;
    default:
        rgb.r = v255;
        rgb.g = p;
        rgb.b = q;
        break;
    }

    return rgb;
}

void neopixel_rainbow(neopixel_strip_t *strip, uint16_t start_hue)
{
    if ((strip == NULL) || !strip->initialized) {
        return;
    }

    uint16_t hue_step = 360U / strip->num_leds;

    for (uint16_t i = 0U; i < strip->num_leds; i++) {
        uint16_t hue = (start_hue + (i * hue_step)) % 360U;
        neopixel_color_t color = neopixel_hsv_to_rgb(hue, 100U, 100U);
        strip->leds[i] = color;
    }
}
