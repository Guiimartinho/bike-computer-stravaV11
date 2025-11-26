/**
 * @file vue.c
 * @brief Display/View management implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "vue/vue.h"
#include "drivers/ls027.h"
#include "model/boucle.h"
#include "model/attitude.h"
#include "model/segment.h"

LOG_MODULE_REGISTER(vue, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Font sizes */
#define FONT_SMALL_H        8U
#define FONT_MEDIUM_H       16U
#define FONT_LARGE_H        32U

/** Screen layout constants */
#define HEADER_HEIGHT       24U
#define FOOTER_HEIGHT       20U
#define CONTENT_START_Y     (HEADER_HEIGHT + 2U)

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Current view mode */
static vue_mode_t current_mode = VUE_MODE_CRS;

/** Current page */
static vue_page_t current_page = VUE_PAGE_MAIN;

/** Active notification */
static notification_t active_notif;
static bool notif_active;
static uint32_t notif_start_time;

/** Initialization flag */
static bool is_initialized;

/* ==========================================================================
 * Private Functions - Drawing Primitives
 * ========================================================================== */

/**
 * @brief Draw a simple character (placeholder - needs font data)
 */
static void draw_char(uint16_t x, uint16_t y, char c, uint8_t size)
{
    /* Simplified character rendering */
    /* Real implementation would use font bitmap data */
    (void)c;

    uint16_t w = (size == 1U) ? 6U : ((size == 2U) ? 12U : 18U);
    uint16_t h = (size == 1U) ? 8U : ((size == 2U) ? 16U : 24U);

    /* Draw character bounding box for visualization */
    for (uint16_t px = 0U; px < w; px++) {
        ls027_draw_pixel(x + px, y, LS027_COLOR_BLACK);
        ls027_draw_pixel(x + px, y + h - 1U, LS027_COLOR_BLACK);
    }
}

/**
 * @brief Draw string
 */
static void draw_string(uint16_t x, uint16_t y, const char *str, uint8_t size)
{
    uint16_t char_w = (size == 1U) ? 6U : ((size == 2U) ? 12U : 18U);
    uint16_t px = x;

    while (*str != '\0') {
        draw_char(px, y, *str, size);
        px += char_w;
        str++;
    }
}

/**
 * @brief Draw horizontal line
 */
static void draw_hline(uint16_t x, uint16_t y, uint16_t len)
{
    ls027_draw_hline(x, y, len, LS027_COLOR_BLACK);
}

/**
 * @brief Draw rectangle outline
 */
static void draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    ls027_draw_hline(x, y, w, LS027_COLOR_BLACK);
    ls027_draw_hline(x, y + h - 1U, w, LS027_COLOR_BLACK);
    ls027_draw_vline(x, y, h, LS027_COLOR_BLACK);
    ls027_draw_vline(x + w - 1U, y, h, LS027_COLOR_BLACK);
}

/* ==========================================================================
 * Private Functions - Screen Rendering
 * ========================================================================== */

/**
 * @brief Draw header bar
 */
static void draw_header(void)
{
    /* Background line */
    draw_hline(0U, HEADER_HEIGHT, LS027_WIDTH);

    /* GPS status indicator */
    draw_string(5U, 4U, "GPS", 1U);

    /* Time display (center) */
    attitude_t att;
    if (attitude_get(&att) == APP_OK) {
        char time_str[16];
        uint32_t secj = att.date.secj;
        uint8_t h = (uint8_t)(secj / 3600U);
        uint8_t m = (uint8_t)((secj % 3600U) / 60U);

        (void)snprintf(time_str, sizeof(time_str), "%02u:%02u", h, m);
        draw_string(180U, 4U, time_str, 1U);
    }

    /* Battery indicator (right) */
    draw_rect(LS027_WIDTH - 30U, 4U, 25U, 12U);
}

/**
 * @brief Draw main cycling page
 */
static void draw_page_main(void)
{
    attitude_t att;
    char buf[32];

    if (attitude_get(&att) != APP_OK) {
        return;
    }

    uint16_t y = CONTENT_START_Y;

    /* Speed - large display */
    (void)snprintf(buf, sizeof(buf), "%.1f", (double)att.loc.speed);
    draw_string(20U, y, buf, 3U);
    draw_string(150U, y + 8U, "km/h", 1U);
    y += 40U;

    /* Distance */
    (void)snprintf(buf, sizeof(buf), "%.2f km", (double)(att.dist / 1000.0f));
    draw_string(20U, y, "Dist:", 1U);
    draw_string(100U, y, buf, 2U);
    y += 24U;

    /* Elevation gain */
    (void)snprintf(buf, sizeof(buf), "%.0f m", (double)att.climb);
    draw_string(20U, y, "Elev:", 1U);
    draw_string(100U, y, buf, 2U);
    y += 24U;

    /* Current slope */
    (void)snprintf(buf, sizeof(buf), "%d%%", att.slope);
    draw_string(20U, y, "Slope:", 1U);
    draw_string(100U, y, buf, 2U);
    y += 24U;

    /* Power */
    (void)snprintf(buf, sizeof(buf), "%u W", att.pwr);
    draw_string(20U, y, "Power:", 1U);
    draw_string(100U, y, buf, 2U);
    y += 24U;

    /* Elapsed time */
    uint32_t secs = attitude_get_elapsed_time();
    uint8_t h = (uint8_t)(secs / 3600U);
    uint8_t m = (uint8_t)((secs % 3600U) / 60U);
    uint8_t s = (uint8_t)(secs % 60U);

    (void)snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h, m, s);
    draw_string(20U, y, "Time:", 1U);
    draw_string(100U, y, buf, 2U);
}

/**
 * @brief Draw segment page
 */
static void draw_page_segment(void)
{
    segment_t seg;
    char buf[32];

    uint16_t y = CONTENT_START_Y;

    if (segment_get_best(&seg) == APP_OK) {
        /* Segment name */
        draw_string(20U, y, seg.name, 2U);
        y += 24U;

        /* Progress */
        (void)snprintf(buf, sizeof(buf), "Progress: %.0f%%", (double)(seg.pct_dist * 100.0f));
        draw_string(20U, y, buf, 1U);
        y += 16U;

        /* Time advantage/deficit */
        const char *sign = (seg.advance >= 0.0f) ? "+" : "";
        (void)snprintf(buf, sizeof(buf), "%s%.1f s", sign, (double)seg.advance);
        draw_string(20U, y, "vs PR:", 1U);
        draw_string(100U, y, buf, 2U);
        y += 24U;

        /* Progress bar */
        uint16_t bar_w = 300U;
        uint16_t bar_h = 20U;
        draw_rect(50U, y, bar_w, bar_h);

        uint16_t fill_w = (uint16_t)((float)bar_w * seg.pct_dist);
        if (fill_w > 0U) {
            ls027_fill_rect(50U, y, fill_w, bar_h, LS027_COLOR_BLACK);
        }
    } else {
        draw_string(100U, 100U, "No active segment", 1U);
    }
}

/**
 * @brief Draw statistics page
 */
static void draw_page_stats(void)
{
    boucle_stats_t stats;
    char buf[32];

    if (boucle_get_stats(&stats) != APP_OK) {
        return;
    }

    uint16_t y = CONTENT_START_Y;

    draw_string(20U, y, "STATISTICS", 2U);
    y += 30U;

    /* Distance */
    (void)snprintf(buf, sizeof(buf), "%.2f km", (double)(stats.total_distance / 1000.0f));
    draw_string(20U, y, "Distance:", 1U);
    draw_string(150U, y, buf, 1U);
    y += 16U;

    /* Climb */
    (void)snprintf(buf, sizeof(buf), "%.0f m", (double)stats.total_climb);
    draw_string(20U, y, "Climb:", 1U);
    draw_string(150U, y, buf, 1U);
    y += 16U;

    /* Max speed */
    (void)snprintf(buf, sizeof(buf), "%.1f km/h", (double)stats.max_speed);
    draw_string(20U, y, "Max Speed:", 1U);
    draw_string(150U, y, buf, 1U);
    y += 16U;

    /* Average speed */
    (void)snprintf(buf, sizeof(buf), "%.1f km/h", (double)stats.avg_speed);
    draw_string(20U, y, "Avg Speed:", 1U);
    draw_string(150U, y, buf, 1U);
    y += 16U;

    /* Moving time */
    uint32_t secs = stats.moving_time;
    uint8_t h = (uint8_t)(secs / 3600U);
    uint8_t m = (uint8_t)((secs % 3600U) / 60U);

    (void)snprintf(buf, sizeof(buf), "%uh %02um", h, m);
    draw_string(20U, y, "Moving:", 1U);
    draw_string(150U, y, buf, 1U);
    y += 16U;

    /* GPS points */
    (void)snprintf(buf, sizeof(buf), "%u", stats.gps_points);
    draw_string(20U, y, "GPS Points:", 1U);
    draw_string(150U, y, buf, 1U);
}

/**
 * @brief Draw notification overlay
 */
static void draw_notification(void)
{
    if (!notif_active) {
        return;
    }

    /* Check timeout */
    uint32_t elapsed = k_uptime_get_32() - notif_start_time;
    if (elapsed >= (active_notif.duration * 1000U)) {
        notif_active = false;
        return;
    }

    /* Draw notification box */
    uint16_t box_w = 300U;
    uint16_t box_h = 60U;
    uint16_t box_x = (LS027_WIDTH - box_w) / 2U;
    uint16_t box_y = 80U;

    /* Clear area */
    ls027_fill_rect(box_x, box_y, box_w, box_h, LS027_COLOR_WHITE);

    /* Draw border */
    draw_rect(box_x, box_y, box_w, box_h);

    /* Draw title */
    draw_string(box_x + 10U, box_y + 8U, active_notif.title, 2U);

    /* Draw message */
    draw_string(box_x + 10U, box_y + 32U, active_notif.message, 1U);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t vue_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    /* Initialize LCD driver */
    app_err_t err = ls027_init();
    if ((err != APP_OK) && (err != APP_ERR_ALREADY_INIT)) {
        LOG_ERR("LCD init failed: %d", err);
        return err;
    }

    /* Clear display */
    ls027_clear();

    current_mode = VUE_MODE_CRS;
    current_page = VUE_PAGE_MAIN;
    notif_active = false;

    is_initialized = true;
    LOG_INF("Vue initialized");

    return APP_OK;
}

void vue_update(void)
{
    if (!is_initialized) {
        return;
    }

    /* Clear buffer */
    ls027_clear();

    /* Draw header */
    draw_header();

    /* Draw current page */
    switch (current_page) {
    case VUE_PAGE_MAIN:
        draw_page_main();
        break;
    case VUE_PAGE_SEGMENT:
        draw_page_segment();
        break;
    case VUE_PAGE_STATS:
        draw_page_stats();
        break;
    case VUE_PAGE_MAP:
    case VUE_PAGE_SENSORS:
    case VUE_PAGE_MENU:
    default:
        draw_string(100U, 100U, "Page not implemented", 1U);
        break;
    }

    /* Draw notification if active */
    draw_notification();

    /* Update display */
    (void)ls027_update();
}

void vue_refresh(void)
{
    vue_update();
}

void vue_set_mode(vue_mode_t mode)
{
    current_mode = mode;
}

vue_mode_t vue_get_mode(void)
{
    return current_mode;
}

void vue_next_page(void)
{
    current_page = (vue_page_t)(((uint8_t)current_page + 1U) % (uint8_t)VUE_PAGE_COUNT);
}

void vue_prev_page(void)
{
    if (current_page == VUE_PAGE_MAIN) {
        current_page = (vue_page_t)((uint8_t)VUE_PAGE_COUNT - 1U);
    } else {
        current_page = (vue_page_t)((uint8_t)current_page - 1U);
    }
}

void vue_set_page(vue_page_t page)
{
    if (page < VUE_PAGE_COUNT) {
        current_page = page;
    }
}

vue_page_t vue_get_page(void)
{
    return current_page;
}

void vue_handle_button(btn_event_t event)
{
    if (!is_initialized) {
        return;
    }

    switch (event) {
    case BTN_EVENT_LEFT:
        vue_prev_page();
        break;

    case BTN_EVENT_RIGHT:
        vue_next_page();
        break;

    case BTN_EVENT_CENTER:
    case BTN_EVENT_LONG_LEFT:
    case BTN_EVENT_LONG_CENTER:
    case BTN_EVENT_LONG_RIGHT:
        /* Handled by boucle */
        break;

    default:
        break;
    }
}

void vue_show_notification(const notification_t *notif)
{
    if ((notif == NULL) || !is_initialized) {
        return;
    }

    active_notif = *notif;
    notif_active = true;
    notif_start_time = k_uptime_get_32();
}

void vue_clear_notification(void)
{
    notif_active = false;
}

void vue_toggle_backlight(void)
{
    /* LS027 is reflective, no backlight */
}

void vue_set_brightness(uint8_t level)
{
    /* LS027 is reflective, no brightness control */
    (void)level;
}
