/**
 * @file main.c
 * @brief stravaV10 - Bike GPS Computer with Strava Segments
 *
 * Main entry point for the Zephyr RTOS based firmware.
 * Follows MISRA C:2012 guidelines.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include "app_types.h"
#include "hal/hal_gpio.h"
#include "hal/hal_spi.h"
#include "hal/hal_i2c.h"
#include "hal/hal_uart.h"
#include "drivers/ls027.h"
#include "drivers/baro.h"
#include "drivers/fxos.h"
#include "drivers/stc3100.h"
#include "drivers/gps_mgmt.h"
#include "model/boucle.h"
#include "rf/ble_manager.h"
#include "vue/vue.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Main loop interval in milliseconds */
#define MAIN_LOOP_INTERVAL_MS       100U

/** Display update interval in milliseconds */
#define DISPLAY_UPDATE_INTERVAL_MS  250U

/** Sensor update interval in milliseconds */
#define SENSOR_UPDATE_INTERVAL_MS   1000U

/** VCOM toggle interval in milliseconds */
#define VCOM_TOGGLE_INTERVAL_MS     1000U

/** Button polling interval in milliseconds */
#define BUTTON_POLL_INTERVAL_MS     20U

/** Watchdog timeout in milliseconds */
#define WDT_TIMEOUT_MS              10000U

/* ==========================================================================
 * Thread Definitions
 * ========================================================================== */

#define MAIN_STACK_SIZE     2048U
#define MAIN_PRIORITY       5

#define DISPLAY_STACK_SIZE  1024U
#define DISPLAY_PRIORITY    7

#define SENSOR_STACK_SIZE   1024U
#define SENSOR_PRIORITY     6

K_THREAD_STACK_DEFINE(main_stack, MAIN_STACK_SIZE);
K_THREAD_STACK_DEFINE(display_stack, DISPLAY_STACK_SIZE);
K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_STACK_SIZE);

static struct k_thread main_thread_data;
static struct k_thread display_thread_data;
static struct k_thread sensor_thread_data;

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** System initialized flag */
static volatile bool system_initialized;

/** Last display update time */
static uint32_t last_display_update;

/** Last sensor update time */
static uint32_t last_sensor_update;

/** Last VCOM toggle time */
static uint32_t last_vcom_toggle;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Initialize all hardware
 */
static app_err_t init_hardware(void)
{
    app_err_t err;

    LOG_INF("Initializing hardware...");

    /* GPIO */
    err = hal_gpio_init();
    if (err != APP_OK) {
        LOG_ERR("GPIO init failed: %d", err);
        return err;
    }

    /* I2C */
    err = hal_i2c_init();
    if (err != APP_OK) {
        LOG_ERR("I2C init failed: %d", err);
        return err;
    }

    /* SPI */
    err = hal_spi_init();
    if (err != APP_OK) {
        LOG_ERR("SPI init failed: %d", err);
        return err;
    }

    /* UART */
    err = hal_uart_init();
    if (err != APP_OK) {
        LOG_ERR("UART init failed: %d", err);
        return err;
    }

    LOG_INF("Hardware initialized");
    return APP_OK;
}

/**
 * @brief Initialize all drivers
 */
static app_err_t init_drivers(void)
{
    app_err_t err;

    LOG_INF("Initializing drivers...");

    /* LCD */
    err = ls027_init();
    if (err != APP_OK) {
        LOG_ERR("LCD init failed: %d", err);
        /* Continue anyway - display might recover */
    }

    /* Barometer */
    err = baro_init();
    if (err != APP_OK) {
        LOG_WRN("Barometer init failed: %d", err);
    }

    /* IMU */
    err = fxos_init();
    if (err != APP_OK) {
        LOG_WRN("IMU init failed: %d", err);
    }

    /* Fuel Gauge */
    battery_config_t batt_cfg = {
        .rsense = 100U,
        .capacity_mah = 1500U,
        .voltage_min = 3300U,
        .voltage_max = 4200U,
    };
    err = stc3100_init(&batt_cfg);
    if (err != APP_OK) {
        LOG_WRN("Fuel gauge init failed: %d", err);
    }

    /* GPS */
    err = gps_mgmt_init();
    if (err != APP_OK) {
        LOG_ERR("GPS init failed: %d", err);
        return err;
    }

    LOG_INF("Drivers initialized");
    return APP_OK;
}

/**
 * @brief Initialize application modules
 */
static app_err_t init_application(void)
{
    app_err_t err;

    LOG_INF("Initializing application...");

    /* BLE Manager */
    err = ble_manager_init();
    if (err != APP_OK) {
        LOG_ERR("BLE init failed: %d", err);
        return err;
    }

    /* Main loop controller */
    err = boucle_init();
    if (err != APP_OK) {
        LOG_ERR("Boucle init failed: %d", err);
        return err;
    }

    /* Display/View system */
    err = vue_init();
    if (err != APP_OK) {
        LOG_ERR("Vue init failed: %d", err);
        return err;
    }

    LOG_INF("Application initialized");
    return APP_OK;
}

/**
 * @brief Button callback handler
 */
static void button_callback(btn_event_t event)
{
    LOG_DBG("Button event: %d", event);

    /* Pass to view system */
    vue_handle_button(event);

    /* Pass to main loop controller */
    boucle_handle_button(event);
}

/**
 * @brief Display update thread
 */
static void display_thread(void *p1, void *p2, void *p3)
{
    (void)p1;
    (void)p2;
    (void)p3;

    LOG_INF("Display thread started");

    while (true) {
        uint32_t now = k_uptime_get_32();

        /* Update display */
        if ((now - last_display_update) >= DISPLAY_UPDATE_INTERVAL_MS) {
            vue_update();
            last_display_update = now;
        }

        /* Toggle VCOM */
        if ((now - last_vcom_toggle) >= VCOM_TOGGLE_INTERVAL_MS) {
            ls027_toggle_vcom();
            last_vcom_toggle = now;
        }

        k_msleep(50);
    }
}

/**
 * @brief Sensor polling thread
 */
static void sensor_thread(void *p1, void *p2, void *p3)
{
    (void)p1;
    (void)p2;
    (void)p3;

    LOG_INF("Sensor thread started");

    while (true) {
        uint32_t now = k_uptime_get_32();

        /* Poll sensors */
        if ((now - last_sensor_update) >= SENSOR_UPDATE_INTERVAL_MS) {
            /* Barometer */
            (void)baro_trigger();

            /* IMU */
            (void)fxos_trigger();

            /* Battery */
            (void)stc3100_trigger();

            /* Update BLE battery level */
            ble_manager_update_battery(stc3100_get_soc());

            last_sensor_update = now;
        }

        k_msleep(100);
    }
}

/**
 * @brief Main application thread
 */
static void main_thread(void *p1, void *p2, void *p3)
{
    (void)p1;
    (void)p2;
    (void)p3;

    LOG_INF("Main thread started");

    /* Wait for system initialization */
    while (!system_initialized) {
        k_msleep(10);
    }

    /* Start GPS */
    (void)gps_mgmt_start();

    /* Start BLE advertising */
    (void)ble_manager_start_advertising();

    /* Load segments from SD card */
    int seg_count = segment_load_all();
    LOG_INF("Loaded %d segments", seg_count);

    /* Set initial mode */
    (void)boucle_set_mode(APP_MODE_CRS);

    /* Main processing loop */
    while (true) {
        /* Process button events */
        hal_gpio_btn_process();

        /* Process GPS data */
        gps_mgmt_process();

        /* Process main loop */
        boucle_process();

        /* Toggle LED to show we're alive */
        static uint32_t led_toggle_time;
        if ((k_uptime_get_32() - led_toggle_time) >= 1000U) {
            hal_gpio_led_toggle();
            led_toggle_time = k_uptime_get_32();
        }

        k_msleep(MAIN_LOOP_INTERVAL_MS);
    }
}

/* ==========================================================================
 * Main Entry Point
 * ========================================================================== */

int main(void)
{
    LOG_INF("=====================================");
    LOG_INF("stravaV10 - Bike GPS Computer");
    LOG_INF("Version %d.%d.%d",
            APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);
    LOG_INF("=====================================");

    /* Initialize hardware */
    app_err_t err = init_hardware();
    if (err != APP_OK) {
        LOG_ERR("Hardware initialization failed!");
        return -1;
    }

    /* Initialize drivers */
    err = init_drivers();
    if (err != APP_OK) {
        LOG_ERR("Driver initialization failed!");
        return -1;
    }

    /* Initialize application */
    err = init_application();
    if (err != APP_OK) {
        LOG_ERR("Application initialization failed!");
        return -1;
    }

    /* Register button callback */
    err = hal_gpio_register_btn_callback(button_callback);
    if (err != APP_OK) {
        LOG_WRN("Button callback registration failed");
    }

    /* Mark system as initialized */
    system_initialized = true;
    LOG_INF("System initialization complete");

    /* Create worker threads */
    k_thread_create(&main_thread_data, main_stack, MAIN_STACK_SIZE,
                    main_thread, NULL, NULL, NULL,
                    MAIN_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&main_thread_data, "main_loop");

    k_thread_create(&display_thread_data, display_stack, DISPLAY_STACK_SIZE,
                    display_thread, NULL, NULL, NULL,
                    DISPLAY_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&display_thread_data, "display");

    k_thread_create(&sensor_thread_data, sensor_stack, SENSOR_STACK_SIZE,
                    sensor_thread, NULL, NULL, NULL,
                    SENSOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thread_data, "sensor");

    LOG_INF("All threads started");

    /* Main function returns, threads continue running */
    return 0;
}
