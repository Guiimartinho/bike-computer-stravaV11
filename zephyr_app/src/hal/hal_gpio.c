/**
 * @file hal_gpio.c
 * @brief GPIO Hardware Abstraction Layer implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "hal/hal_gpio.h"

LOG_MODULE_REGISTER(hal_gpio, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Button debounce time in milliseconds */
#define BTN_DEBOUNCE_MS         50U

/** Long press threshold in milliseconds */
#define BTN_LONG_PRESS_MS       1000U

/** Button state structure */
typedef struct {
    bool current_state;
    bool last_state;
    uint32_t press_time;
    bool long_press_sent;
} btn_state_t;

/* ==========================================================================
 * Device Tree Bindings
 * ========================================================================== */

/* LED */
static const struct gpio_dt_spec led_status = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Buttons */
static const struct gpio_dt_spec btn_left = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn_center = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec btn_right = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

/* GPS Control */
static const struct gpio_dt_spec gps_reset = GPIO_DT_SPEC_GET(DT_NODELABEL(gps_reset), gpios);
static const struct gpio_dt_spec gps_stdby = GPIO_DT_SPEC_GET(DT_NODELABEL(gps_stdby), gpios);
static const struct gpio_dt_spec gps_fix = GPIO_DT_SPEC_GET(DT_NODELABEL(gps_fix), gpios);

/* IMU Control */
static const struct gpio_dt_spec imu_int1 = GPIO_DT_SPEC_GET(DT_NODELABEL(imu_int1), gpios);
static const struct gpio_dt_spec imu_reset = GPIO_DT_SPEC_GET(DT_NODELABEL(imu_reset), gpios);

/* Neopixel */
static const struct gpio_dt_spec neopixel = GPIO_DT_SPEC_GET(DT_NODELABEL(neo_data), gpios);

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** GPIO pin mapping table */
static const struct gpio_dt_spec *gpio_pins[HAL_GPIO_PIN_COUNT] = {
    [HAL_GPIO_LED_STATUS] = &led_status,
    [HAL_GPIO_BTN_LEFT]   = &btn_left,
    [HAL_GPIO_BTN_CENTER] = &btn_center,
    [HAL_GPIO_BTN_RIGHT]  = &btn_right,
    [HAL_GPIO_GPS_RESET]  = &gps_reset,
    [HAL_GPIO_GPS_STDBY]  = &gps_stdby,
    [HAL_GPIO_GPS_FIX]    = &gps_fix,
    [HAL_GPIO_IMU_INT1]   = &imu_int1,
    [HAL_GPIO_IMU_RESET]  = &imu_reset,
    [HAL_GPIO_NEOPIXEL]   = &neopixel,
};

/** Button states for debouncing */
static btn_state_t btn_states[3];

/** Button callback function */
static hal_gpio_btn_callback_t btn_callback;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Check if pin is valid and initialized
 */
static bool is_pin_valid(hal_gpio_pin_t pin)
{
    if (pin >= HAL_GPIO_PIN_COUNT) {
        return false;
    }
    if (gpio_pins[pin] == NULL) {
        return false;
    }
    return gpio_is_ready_dt(gpio_pins[pin]);
}

/**
 * @brief Process single button state
 */
static void process_button(uint8_t btn_idx, hal_gpio_pin_t pin, btn_event_t short_evt,
                          btn_event_t long_evt)
{
    bool state = false;
    btn_state_t *btn = &btn_states[btn_idx];
    uint32_t now = k_uptime_get_32();

    if (hal_gpio_get(pin, &state) != APP_OK) {
        return;
    }

    /* Button is active low, invert logic */
    state = !state;

    if (state && !btn->last_state) {
        /* Button just pressed */
        btn->press_time = now;
        btn->long_press_sent = false;
    } else if (state && btn->last_state) {
        /* Button held */
        if (!btn->long_press_sent &&
            ((now - btn->press_time) >= BTN_LONG_PRESS_MS)) {
            btn->long_press_sent = true;
            if (btn_callback != NULL) {
                btn_callback(long_evt);
            }
        }
    } else if (!state && btn->last_state) {
        /* Button released */
        if (!btn->long_press_sent) {
            if (btn_callback != NULL) {
                btn_callback(short_evt);
            }
        }
    }

    btn->last_state = state;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t hal_gpio_init(void)
{
    int ret;

    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Configure LED as output */
    if (!gpio_is_ready_dt(&led_status)) {
        LOG_ERR("LED GPIO not ready");
        return APP_ERR_NOT_INIT;
    }
    ret = gpio_pin_configure_dt(&led_status, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED: %d", ret);
        return APP_ERR_IO;
    }

    /* Configure buttons as inputs */
    if (!gpio_is_ready_dt(&btn_left) ||
        !gpio_is_ready_dt(&btn_center) ||
        !gpio_is_ready_dt(&btn_right)) {
        LOG_ERR("Button GPIOs not ready");
        return APP_ERR_NOT_INIT;
    }

    ret = gpio_pin_configure_dt(&btn_left, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button left: %d", ret);
        return APP_ERR_IO;
    }

    ret = gpio_pin_configure_dt(&btn_center, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button center: %d", ret);
        return APP_ERR_IO;
    }

    ret = gpio_pin_configure_dt(&btn_right, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button right: %d", ret);
        return APP_ERR_IO;
    }

    /* Configure GPS control pins */
    if (gpio_is_ready_dt(&gps_reset)) {
        ret = gpio_pin_configure_dt(&gps_reset, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure GPS reset: %d", ret);
        }
    }

    if (gpio_is_ready_dt(&gps_stdby)) {
        ret = gpio_pin_configure_dt(&gps_stdby, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure GPS standby: %d", ret);
        }
    }

    if (gpio_is_ready_dt(&gps_fix)) {
        ret = gpio_pin_configure_dt(&gps_fix, GPIO_INPUT);
        if (ret < 0) {
            LOG_WRN("Failed to configure GPS fix: %d", ret);
        }
    }

    /* Configure IMU control pins */
    if (gpio_is_ready_dt(&imu_int1)) {
        ret = gpio_pin_configure_dt(&imu_int1, GPIO_INPUT);
        if (ret < 0) {
            LOG_WRN("Failed to configure IMU INT1: %d", ret);
        }
    }

    if (gpio_is_ready_dt(&imu_reset)) {
        ret = gpio_pin_configure_dt(&imu_reset, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure IMU reset: %d", ret);
        }
    }

    /* Configure Neopixel pin */
    if (gpio_is_ready_dt(&neopixel)) {
        ret = gpio_pin_configure_dt(&neopixel, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure Neopixel: %d", ret);
        }
    }

    /* Initialize button states */
    (void)memset(btn_states, 0, sizeof(btn_states));

    is_initialized = true;
    LOG_INF("GPIO HAL initialized");

    return APP_OK;
}

app_err_t hal_gpio_set(hal_gpio_pin_t pin, bool state)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (!is_pin_valid(pin)) {
        return APP_ERR_INVALID_PARAM;
    }

    int ret = gpio_pin_set_dt(gpio_pins[pin], state ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to set GPIO %d: %d", pin, ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_gpio_get(hal_gpio_pin_t pin, bool *state)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (!is_pin_valid(pin) || (state == NULL)) {
        return APP_ERR_INVALID_PARAM;
    }

    int ret = gpio_pin_get_dt(gpio_pins[pin]);
    if (ret < 0) {
        LOG_ERR("Failed to get GPIO %d: %d", pin, ret);
        return APP_ERR_IO;
    }

    *state = (ret != 0);
    return APP_OK;
}

app_err_t hal_gpio_toggle(hal_gpio_pin_t pin)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    if (!is_pin_valid(pin)) {
        return APP_ERR_INVALID_PARAM;
    }

    int ret = gpio_pin_toggle_dt(gpio_pins[pin]);
    if (ret < 0) {
        LOG_ERR("Failed to toggle GPIO %d: %d", pin, ret);
        return APP_ERR_IO;
    }

    return APP_OK;
}

app_err_t hal_gpio_register_btn_callback(hal_gpio_btn_callback_t callback)
{
    if (!is_initialized) {
        return APP_ERR_NOT_INIT;
    }

    btn_callback = callback;
    return APP_OK;
}

void hal_gpio_btn_process(void)
{
    if (!is_initialized) {
        return;
    }

    process_button(0, HAL_GPIO_BTN_LEFT, BTN_EVENT_LEFT, BTN_EVENT_LONG_LEFT);
    process_button(1, HAL_GPIO_BTN_CENTER, BTN_EVENT_CENTER, BTN_EVENT_LONG_CENTER);
    process_button(2, HAL_GPIO_BTN_RIGHT, BTN_EVENT_RIGHT, BTN_EVENT_LONG_RIGHT);
}

void hal_gpio_led_set(bool on)
{
    (void)hal_gpio_set(HAL_GPIO_LED_STATUS, on);
}

void hal_gpio_led_toggle(void)
{
    (void)hal_gpio_toggle(HAL_GPIO_LED_STATUS);
}
