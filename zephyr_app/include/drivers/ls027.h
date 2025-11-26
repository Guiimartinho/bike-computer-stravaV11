/**
 * @file ls027.h
 * @brief Sharp Memory LCD LS027B4DH01 driver for stravaV10
 *
 * Driver for 400x240 monochrome Sharp Memory LCD.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef DRIVERS_LS027_H
#define DRIVERS_LS027_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** Display dimensions */
#define LS027_WIDTH             400U
#define LS027_HEIGHT            240U

/** Pixel colors */
#define LS027_COLOR_BLACK       1U
#define LS027_COLOR_WHITE       0U
#define LS027_COLOR_INVERT      2U  /**< XOR with current pixel */

/** Buffer size in bytes (1 bit per pixel) */
#define LS027_BUFFER_SIZE       ((LS027_WIDTH * LS027_HEIGHT) / 8U)

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Display orientation */
typedef enum {
    LS027_ORIENT_LANDSCAPE = 0,     /**< Normal landscape (400x240) */
    LS027_ORIENT_LANDSCAPE_180,     /**< Rotated 180 degrees */
    LS027_ORIENT_PORTRAIT,          /**< Portrait mode (240x400) */
    LS027_ORIENT_PORTRAIT_180       /**< Portrait rotated 180 */
} ls027_orient_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize LCD driver
 * @return APP_OK on success, error code otherwise
 */
app_err_t ls027_init(void);

/**
 * @brief Clear display buffer
 */
void ls027_clear(void);

/**
 * @brief Update entire display from buffer
 * @return APP_OK on success, error code otherwise
 */
app_err_t ls027_update(void);

/**
 * @brief Toggle VCOM signal (call periodically ~1Hz)
 *
 * Required to prevent DC bias buildup on LCD.
 */
void ls027_toggle_vcom(void);

/**
 * @brief Draw single pixel
 * @param x X coordinate (0 to LS027_WIDTH-1)
 * @param y Y coordinate (0 to LS027_HEIGHT-1)
 * @param color Pixel color (LS027_COLOR_BLACK, WHITE, or INVERT)
 */
void ls027_draw_pixel(uint16_t x, uint16_t y, uint8_t color);

/**
 * @brief Draw horizontal line of pixels (optimized)
 * @param x Starting X coordinate
 * @param y Y coordinate
 * @param len Number of pixels
 * @param color Pixel color
 */
void ls027_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint8_t color);

/**
 * @brief Draw vertical line of pixels
 * @param x X coordinate
 * @param y Starting Y coordinate
 * @param len Number of pixels
 * @param color Pixel color
 */
void ls027_draw_vline(uint16_t x, uint16_t y, uint16_t len, uint8_t color);

/**
 * @brief Fill rectangle
 * @param x Starting X coordinate
 * @param y Starting Y coordinate
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Fill color
 */
void ls027_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);

/**
 * @brief Get pointer to display buffer
 * @return Pointer to internal framebuffer
 */
uint8_t *ls027_get_buffer(void);

/**
 * @brief Invert display colors
 */
void ls027_invert_colors(void);

/**
 * @brief Set display orientation
 * @param orient Display orientation
 */
void ls027_set_orientation(ls027_orient_t orient);

/**
 * @brief Get current display width (depends on orientation)
 * @return Display width in pixels
 */
uint16_t ls027_get_width(void);

/**
 * @brief Get current display height (depends on orientation)
 * @return Display height in pixels
 */
uint16_t ls027_get_height(void);

/**
 * @brief Check if display is busy
 * @return true if display update in progress
 */
bool ls027_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_LS027_H */
