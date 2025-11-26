/**
 * @file hal_gpio.h
 * @brief GPIO Hardware Abstraction Layer for stravaV10
 *
 * Provides abstracted GPIO operations for buttons, LEDs, and control pins.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** GPIO pin identifiers */
typedef enum {
    HAL_GPIO_LED_STATUS = 0,
    HAL_GPIO_BTN_LEFT,
    HAL_GPIO_BTN_CENTER,
    HAL_GPIO_BTN_RIGHT,
    HAL_GPIO_GPS_RESET,
    HAL_GPIO_GPS_STDBY,
    HAL_GPIO_GPS_FIX,
    HAL_GPIO_IMU_INT1,
    HAL_GPIO_IMU_RESET,
    HAL_GPIO_NEOPIXEL,
    HAL_GPIO_PIN_COUNT
} hal_gpio_pin_t;

/** Button callback function type */
typedef void (*hal_gpio_btn_callback_t)(btn_event_t event);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize GPIO subsystem
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_gpio_init(void);

/**
 * @brief Set GPIO pin state
 * @param pin Pin identifier
 * @param state Pin state (true = high, false = low)
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_gpio_set(hal_gpio_pin_t pin, bool state);

/**
 * @brief Get GPIO pin state
 * @param pin Pin identifier
 * @param state Pointer to store pin state
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_gpio_get(hal_gpio_pin_t pin, bool *state);

/**
 * @brief Toggle GPIO pin state
 * @param pin Pin identifier
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_gpio_toggle(hal_gpio_pin_t pin);

/**
 * @brief Register button event callback
 * @param callback Function to call on button events
 * @return APP_OK on success, error code otherwise
 */
app_err_t hal_gpio_register_btn_callback(hal_gpio_btn_callback_t callback);

/**
 * @brief Process button debouncing (call from timer)
 */
void hal_gpio_btn_process(void);

/**
 * @brief Set LED state
 * @param on LED state
 */
void hal_gpio_led_set(bool on);

/**
 * @brief Toggle LED
 */
void hal_gpio_led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
