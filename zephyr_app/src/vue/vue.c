/**
 * @file vue.c
 * @brief Display/View management implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "vue/vue.h"
#include "drivers/ls027.h"
#include "model/boucle.h"
#include "model/attitude.h"
#include "model/segment.h"
#include "model/parcours.h"
#include "drivers/gps_mgmt.h"

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
 * @brief Draw parcours/route navigation page
 */
static void draw_page_parcours(void)
{
    parcours_info_t pinfo;
    nav_info_t nav;
    char buf[32];

    uint16_t y = CONTENT_START_Y;

    if (!parcours_is_loaded()) {
        draw_string(60U, 100U, "No route loaded", 2U);
        draw_string(40U, 130U, "Load .CRS from SD card", 1U);
        return;
    }

    if (parcours_get_info(&pinfo) != APP_OK) {
        return;
    }

    /* Route name */
    draw_string(20U, y, pinfo.name, 2U);
    y += 28U;

    draw_hline(20U, y, LS027_WIDTH - 40U);
    y += 8U;

    if (!parcours_is_active()) {
        draw_string(60U, y + 30U, "Press START to begin", 1U);
        return;
    }

    if (parcours_get_nav_info(&nav) != APP_OK) {
        return;
    }

    /* Progress percentage */
    (void)snprintf(buf, sizeof(buf), "Progress: %.0f%%", (double)nav.pct_complete);
    draw_string(20U, y, buf, 2U);
    y += 24U;

    /* Progress bar */
    uint16_t bar_w = LS027_WIDTH - 60U;
    uint16_t bar_h = 16U;
    draw_rect(30U, y, bar_w, bar_h);
    uint16_t fill_w = (uint16_t)((float)bar_w * (nav.pct_complete / 100.0f));
    if (fill_w > 0U) {
        ls027_fill_rect(30U, y, fill_w, bar_h, LS027_COLOR_BLACK);
    }
    y += 24U;

    /* Distance remaining */
    (void)snprintf(buf, sizeof(buf), "%.2f km", (double)(nav.dist_remaining / 1000.0f));
    draw_string(20U, y, "Remaining:", 1U);
    draw_string(140U, y, buf, 2U);
    y += 24U;

    /* Distance to route */
    if (nav.on_route) {
        (void)snprintf(buf, sizeof(buf), "%.0f m", (double)nav.dist_to_route);
    } else {
        (void)snprintf(buf, sizeof(buf), "OFF ROUTE (%.0f m)", (double)nav.dist_to_route);
    }
    draw_string(20U, y, "To Route:", 1U);
    draw_string(140U, y, buf, 1U);
    y += 20U;

    /* Bearing to next point */
    (void)snprintf(buf, sizeof(buf), "%.0f", (double)nav.bearing);
    draw_string(20U, y, "Bearing:", 1U);
    draw_string(140U, y, buf, 2U);
    y += 24U;

    /* Next point altitude */
    (void)snprintf(buf, sizeof(buf), "%.0f m", (double)nav.altitude_next);
    draw_string(20U, y, "Next Alt:", 1U);
    draw_string(140U, y, buf, 1U);
}

/**
 * @brief Draw GPS debug page
 */
static void draw_page_gps(void)
{
    gps_data_t gps;
    char buf[48];

    uint16_t y = CONTENT_START_Y;

    draw_string(100U, y, "GPS STATUS", 2U);
    y += 30U;

    gps_state_t state = gps_mgmt_get_state();
    const char *state_str;
    switch (state) {
    case GPS_STATE_OFF:
        state_str = "OFF";
        break;
    case GPS_STATE_INIT:
        state_str = "INIT";
        break;
    case GPS_STATE_ACQUIRING:
        state_str = "ACQUIRING";
        break;
    case GPS_STATE_FIX_2D:
        state_str = "FIX 2D";
        break;
    case GPS_STATE_FIX_3D:
        state_str = "FIX 3D";
        break;
    case GPS_STATE_STANDBY:
        state_str = "STANDBY";
        break;
    default:
        state_str = "UNKNOWN";
        break;
    }

    (void)snprintf(buf, sizeof(buf), "State: %s", state_str);
    draw_string(20U, y, buf, 1U);
    y += 16U;

    if (gps_mgmt_get_data(&gps) == APP_OK) {
        /* Satellites */
        (void)snprintf(buf, sizeof(buf), "Satellites: %u", gps.satellites);
        draw_string(20U, y, buf, 1U);
        y += 16U;

        /* HDOP */
        (void)snprintf(buf, sizeof(buf), "HDOP: %.1f", (double)gps.hdop);
        draw_string(20U, y, buf, 1U);
        y += 16U;

        if (gps.fix_valid) {
            /* Latitude */
            (void)snprintf(buf, sizeof(buf), "Lat:  %.6f", (double)gps.location.lat);
            draw_string(20U, y, buf, 1U);
            y += 16U;

            /* Longitude */
            (void)snprintf(buf, sizeof(buf), "Lon:  %.6f", (double)gps.location.lon);
            draw_string(20U, y, buf, 1U);
            y += 16U;

            /* Altitude */
            (void)snprintf(buf, sizeof(buf), "Alt:  %.1f m", (double)gps.location.alt);
            draw_string(20U, y, buf, 1U);
            y += 16U;

            /* Speed */
            (void)snprintf(buf, sizeof(buf), "Speed: %.1f km/h", (double)gps.location.speed);
            draw_string(20U, y, buf, 1U);
            y += 16U;

            /* Course */
            (void)snprintf(buf, sizeof(buf), "Course: %.1f", (double)gps.location.course);
            draw_string(20U, y, buf, 1U);
        } else {
            draw_string(60U, y + 20U, "No valid fix", 2U);
        }
    } else {
        draw_string(60U, y + 20U, "GPS data unavailable", 1U);
    }
}

/**
 * @brief Draw line using Bresenham's algorithm
 */
static void draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    int16_t dx = (int16_t)((x1 > x0) ? (x1 - x0) : (x0 - x1));
    int16_t dy = (int16_t)((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    int16_t x = (int16_t)x0;
    int16_t y = (int16_t)y0;

    while (true) {
        if ((x >= 0) && (x < (int16_t)LS027_WIDTH) &&
            (y >= 0) && (y < (int16_t)LS027_HEIGHT)) {
            ls027_draw_pixel((uint16_t)x, (uint16_t)y, LS027_COLOR_BLACK);
        }

        if ((x == (int16_t)x1) && (y == (int16_t)y1)) {
            break;
        }

        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

/**
 * @brief Draw a simple filled circle (for map markers)
 */
static void draw_filled_circle(uint16_t cx, uint16_t cy, uint16_t r)
{
    for (int16_t y = -(int16_t)r; y <= (int16_t)r; y++) {
        for (int16_t x = -(int16_t)r; x <= (int16_t)r; x++) {
            if ((x * x + y * y) <= (int16_t)(r * r)) {
                uint16_t px = (uint16_t)((int16_t)cx + x);
                uint16_t py = (uint16_t)((int16_t)cy + y);
                if ((px < LS027_WIDTH) && (py < LS027_HEIGHT)) {
                    ls027_draw_pixel(px, py, LS027_COLOR_BLACK);
                }
            }
        }
    }
}

/**
 * @brief Draw a circle outline
 */
static void draw_circle(uint16_t cx, uint16_t cy, uint16_t r)
{
    int16_t x = (int16_t)r;
    int16_t y = 0;
    int16_t err = 0;

    while (x >= y) {
        ls027_draw_pixel(cx + (uint16_t)x, cy + (uint16_t)y, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx + (uint16_t)y, cy + (uint16_t)x, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx - (uint16_t)x, cy + (uint16_t)y, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx - (uint16_t)y, cy + (uint16_t)x, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx - (uint16_t)x, cy - (uint16_t)y, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx - (uint16_t)y, cy - (uint16_t)x, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx + (uint16_t)x, cy - (uint16_t)y, LS027_COLOR_BLACK);
        ls027_draw_pixel(cx + (uint16_t)y, cy - (uint16_t)x, LS027_COLOR_BLACK);

        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        }
        if (err > 0) {
            x--;
            err -= 2 * x + 1;
        }
    }
}

/**
 * @brief Draw compass arrow pointing in direction
 */
static void draw_compass_arrow(uint16_t cx, uint16_t cy, uint16_t len, float bearing_deg)
{
    /* Convert bearing to radians, adjust for screen coords (0=up, CW positive) */
    float rad = (bearing_deg - 90.0f) * 3.14159265f / 180.0f;

    /* Arrow tip */
    int16_t tip_x = (int16_t)cx + (int16_t)((float)len * cosf(rad));
    int16_t tip_y = (int16_t)cy + (int16_t)((float)len * sinf(rad));

    /* Arrow base (opposite direction) */
    int16_t base_x = (int16_t)cx - (int16_t)(((float)len / 3.0f) * cosf(rad));
    int16_t base_y = (int16_t)cy - (int16_t)(((float)len / 3.0f) * sinf(rad));

    /* Draw arrow line */
    draw_line((uint16_t)base_x, (uint16_t)base_y,
              (uint16_t)tip_x, (uint16_t)tip_y);

    /* Draw arrowhead */
    float head_angle = 0.5f;  /* radians offset for head */
    int16_t h1_x = tip_x - (int16_t)(10.0f * cosf(rad - head_angle));
    int16_t h1_y = tip_y - (int16_t)(10.0f * sinf(rad - head_angle));
    int16_t h2_x = tip_x - (int16_t)(10.0f * cosf(rad + head_angle));
    int16_t h2_y = tip_y - (int16_t)(10.0f * sinf(rad + head_angle));

    draw_line((uint16_t)tip_x, (uint16_t)tip_y, (uint16_t)h1_x, (uint16_t)h1_y);
    draw_line((uint16_t)tip_x, (uint16_t)tip_y, (uint16_t)h2_x, (uint16_t)h2_y);
}

/**
 * @brief Draw map page with position and nearby info
 */
static void draw_page_map(void)
{
    attitude_t att;
    char buf[32];

    uint16_t y = CONTENT_START_Y;

    draw_string(130U, y, "MAP VIEW", 2U);
    y += 28U;

    if (attitude_get(&att) != APP_OK) {
        draw_string(80U, 100U, "No position data", 1U);
        return;
    }

    /* Map area - center of screen */
    uint16_t map_cx = LS027_WIDTH / 2U;
    uint16_t map_cy = 120U;
    uint16_t map_radius = 70U;

    /* Draw map boundary circle */
    draw_circle(map_cx, map_cy, map_radius);
    draw_circle(map_cx, map_cy, map_radius + 1U);

    /* Draw current position marker (center) */
    draw_filled_circle(map_cx, map_cy, 6U);

    /* Draw course/heading arrow if moving */
    if (att.loc.speed > 2.0f) {
        draw_compass_arrow(map_cx, map_cy, 50U, att.loc.course);
    }

    /* Draw cardinal directions */
    draw_string(map_cx - 3U, map_cy - map_radius - 12U, "N", 1U);
    draw_string(map_cx - 3U, map_cy + map_radius + 4U, "S", 1U);
    draw_string(map_cx - map_radius - 10U, map_cy - 4U, "W", 1U);
    draw_string(map_cx + map_radius + 4U, map_cy - 4U, "E", 1U);

    /* Info below map */
    y = map_cy + map_radius + 25U;

    /* Coordinates */
    (void)snprintf(buf, sizeof(buf), "%.5f, %.5f",
                   (double)att.loc.lat, (double)att.loc.lon);
    draw_string(60U, y, buf, 1U);
    y += 14U;

    /* Altitude */
    (void)snprintf(buf, sizeof(buf), "Alt: %.0f m", (double)att.loc.alt);
    draw_string(20U, y, buf, 1U);

    /* Speed */
    (void)snprintf(buf, sizeof(buf), "%.1f km/h", (double)att.loc.speed);
    draw_string(180U, y, buf, 1U);
    y += 14U;

    /* Course */
    (void)snprintf(buf, sizeof(buf), "Course: %.0f", (double)att.loc.course);
    draw_string(20U, y, buf, 1U);

    /* Distance to nearest segment */
    float seg_dist = segment_get_nearest_distance();
    if (seg_dist >= 0.0f) {
        (void)snprintf(buf, sizeof(buf), "Seg: %.0fm", (double)seg_dist);
        draw_string(180U, y, buf, 1U);
    }
}

/**
 * @brief Draw debug info page
 */
static void draw_page_debug(void)
{
    char buf[48];
    uint16_t y = CONTENT_START_Y;

    draw_string(100U, y, "DEBUG INFO", 2U);
    y += 30U;

    /* Uptime */
    uint32_t uptime_ms = k_uptime_get_32();
    uint32_t secs = uptime_ms / 1000U;
    uint32_t mins = secs / 60U;
    uint32_t hours = mins / 60U;

    (void)snprintf(buf, sizeof(buf), "Uptime: %02u:%02u:%02u",
                   (unsigned)(hours % 100U),
                   (unsigned)(mins % 60U),
                   (unsigned)(secs % 60U));
    draw_string(20U, y, buf, 1U);
    y += 16U;

    /* Heap configured size */
    (void)snprintf(buf, sizeof(buf), "Heap: %u bytes configured",
                   (unsigned)CONFIG_HEAP_MEM_POOL_SIZE);
    draw_string(20U, y, buf, 1U);
    y += 16U;

    /* BLE state */
    draw_string(20U, y, "BLE:", 1U);
    draw_string(100U, y, "Active", 1U);
    y += 16U;

    /* Boucle state */
    boucle_state_t bstate = boucle_get_state();
    const char *bstate_str;
    switch (bstate) {
    case BOUCLE_STATE_IDLE:
        bstate_str = "IDLE";
        break;
    case BOUCLE_STATE_RUNNING:
        bstate_str = "RUNNING";
        break;
    case BOUCLE_STATE_PAUSED:
        bstate_str = "PAUSED";
        break;
    default:
        bstate_str = "UNKNOWN";
        break;
    }

    (void)snprintf(buf, sizeof(buf), "Boucle: %s", bstate_str);
    draw_string(20U, y, buf, 1U);
    y += 16U;

    /* Version */
    (void)snprintf(buf, sizeof(buf), "Version: %u.%u.%u",
                   APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);
    draw_string(20U, y, buf, 1U);
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
    case VUE_PAGE_PARCOURS:
        draw_page_parcours();
        break;
    case VUE_PAGE_STATS:
        draw_page_stats();
        break;
    case VUE_PAGE_GPS:
        draw_page_gps();
        break;
    case VUE_PAGE_DEBUG:
        draw_page_debug();
        break;
    case VUE_PAGE_MAP:
        draw_page_map();
        break;
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
