# SPDX-License-Identifier: Apache-2.0
# stravaV10 - Zephyr Application
# Bike GPS Computer with Strava Segments

cmake_minimum_required(VERSION 3.20.0)

# Board configuration
set(BOARD nrf52840dk/nrf52840)
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/nrf52840_strava.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(stravaV10 VERSION 2.0.0 LANGUAGES C)

# Compile options following MISRA and 0.memory standards
target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Werror=implicit-function-declaration
    -Werror=return-type
    -Wno-unused-parameter
    -ffunction-sections
    -fdata-sections
)

# Include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rf
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vue
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
)

# =============================================================================
# Source files organized by module
# =============================================================================

# Main entry point
target_sources(app PRIVATE
    src/main.c
)

# HAL Layer - Hardware Abstraction
target_sources(app PRIVATE
    src/hal/hal_gpio.c
    src/hal/hal_spi.c
    src/hal/hal_i2c.c
    src/hal/hal_uart.c
)

# Drivers - LCD
target_sources(app PRIVATE
    src/drivers/lcd/ls027.c
)

# Drivers - Sensors
target_sources(app PRIVATE
    src/drivers/sensors/baro.c
    src/drivers/sensors/fxos.c
    src/drivers/sensors/stc3100.c
)

# Drivers - GPS
target_sources(app PRIVATE
    src/drivers/gps/gps_mgmt.c
    src/drivers/gps/nmea_parser.c
)

# Drivers - NeoPixel
target_sources(app PRIVATE
    src/drivers/neopixel.c
)

# Model Layer - Business Logic
target_sources(app PRIVATE
    src/model/boucle.c
    src/model/attitude.c
    src/model/locator.c
    src/model/segment.c
    src/model/kalman.c
    src/model/kalman_altitude.c
    src/model/udmatrix.c
    src/model/vecteur.c
    src/model/liste_points.c
    src/model/power_zone.c
    src/model/suffer_score.c
    src/model/rr_zone.c
    src/model/user_settings.c
    src/model/crash_recovery.c
    src/model/parcours.c
    src/model/baro_drift.c
    src/model/loc_source.c
    src/model/sd_logger.c
)

# RF Layer - Wireless Communication
target_sources(app PRIVATE
    src/rf/ble/ble_manager.c
    src/rf/ble/ble_nus.c
    src/rf/ble/ble_lns.c
    src/rf/ble_hrs_client.c
    src/rf/ble_bsc_client.c
    src/rf/ble_fec_client.c
    src/rf/ble_komoot_client.c
)

# Vue Layer - User Interface
target_sources(app PRIVATE
    src/vue/vue.c
    src/vue/vue_crs.c
    src/vue/vue_fec.c
    src/vue/menu.c
)

# Utils
target_sources(app PRIVATE
    src/utils/utils.c
    src/utils/ring_buffer.c
)
