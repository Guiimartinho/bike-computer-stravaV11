/**
 * @file menu.c
 * @brief Menu system implementation
 *
 * Provides hierarchical menu navigation for user settings.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "vue/menu.h"
#include "vue/vue.h"
#include "model/user_settings.h"
#include "model/boucle.h"

LOG_MODULE_REGISTER(menu, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Visible menu items on screen */
#define VISIBLE_ITEMS       5U

/* ==========================================================================
 * Action Callbacks
 * ========================================================================== */

static void action_start_ride(void)
{
    boucle_start();
    menu_close();
    LOG_INF("Ride started from menu");
}

static void action_pause_ride(void)
{
    boucle_pause();
    LOG_INF("Ride paused from menu");
}

static void action_stop_ride(void)
{
    boucle_stop();
    menu_close();
    LOG_INF("Ride stopped from menu");
}

static void action_save_settings(void)
{
    user_settings_t *settings = user_settings_get_global();
    (void)user_settings_save(settings);
    LOG_INF("Settings saved");
}

static void action_reset_trip(void)
{
    boucle_reset_stats();
    LOG_INF("Trip reset from menu");
}

/* ==========================================================================
 * Menu Definitions
 * ========================================================================== */

/* Settings submenu */
static user_settings_t *g_settings;

static const menu_item_t settings_menu[] = {
    { "FTP (watts)",    MENU_TYPE_VALUE,  NULL, NULL, 100U, 500U, 5U, NULL, 0U },
    { "Weight (kg)",    MENU_TYPE_VALUE,  NULL, NULL, 400U, 1500U, 10U, NULL, 0U },
    { "Save Settings",  MENU_TYPE_ACTION, action_save_settings, NULL, 0U, 0U, 0U, NULL, 0U },
    { "< Back",         MENU_TYPE_BACK,   NULL, NULL, 0U, 0U, 0U, NULL, 0U }
};

/* Ride control submenu */
static const menu_item_t ride_menu[] = {
    { "Start Ride",     MENU_TYPE_ACTION, action_start_ride, NULL, 0U, 0U, 0U, NULL, 0U },
    { "Pause Ride",     MENU_TYPE_ACTION, action_pause_ride, NULL, 0U, 0U, 0U, NULL, 0U },
    { "Stop Ride",      MENU_TYPE_ACTION, action_stop_ride, NULL, 0U, 0U, 0U, NULL, 0U },
    { "Reset Trip",     MENU_TYPE_ACTION, action_reset_trip, NULL, 0U, 0U, 0U, NULL, 0U },
    { "< Back",         MENU_TYPE_BACK,   NULL, NULL, 0U, 0U, 0U, NULL, 0U }
};

/* Main menu */
static const menu_item_t main_menu[] = {
    { "Ride Control",   MENU_TYPE_SUBMENU, NULL, NULL, 0U, 0U, 0U, ride_menu, 5U },
    { "Settings",       MENU_TYPE_SUBMENU, NULL, NULL, 0U, 0U, 0U, settings_menu, 4U },
    { "Close Menu",     MENU_TYPE_ACTION,  menu_close, NULL, 0U, 0U, 0U, NULL, 0U }
};

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Menu state */
static menu_state_t state;

/** Menu active flag */
static bool is_active;

/** Initialization flag */
static bool is_initialized;

/** Temporary value for editing */
static uint16_t edit_value;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t menu_init(void)
{
    if (is_initialized) {
        return APP_ERR_ALREADY_INIT;
    }

    (void)memset(&state, 0, sizeof(state));

    state.current_menu = main_menu;
    state.menu_count = 3U;
    state.selected_index = 0U;
    state.scroll_offset = 0U;
    state.editing_value = false;
    state.depth = 0U;

    g_settings = user_settings_get_global();

    is_active = false;
    is_initialized = true;

    LOG_INF("Menu system initialized");
    return APP_OK;
}

bool menu_is_active(void)
{
    return is_active;
}

void menu_open(void)
{
    if (!is_initialized) {
        (void)menu_init();
    }

    state.current_menu = main_menu;
    state.menu_count = 3U;
    state.selected_index = 0U;
    state.scroll_offset = 0U;
    state.editing_value = false;
    state.depth = 0U;

    is_active = true;
    LOG_INF("Menu opened");
}

void menu_close(void)
{
    is_active = false;
    state.editing_value = false;
    LOG_INF("Menu closed");
}

void menu_up(void)
{
    if (!is_active) {
        return;
    }

    if (state.editing_value) {
        /* Increase value */
        const menu_item_t *item = &state.current_menu[state.selected_index];
        if ((edit_value + item->value_step) <= item->value_max) {
            edit_value += item->value_step;
        }
    } else {
        /* Navigate up */
        if (state.selected_index > 0U) {
            state.selected_index--;
            if (state.selected_index < state.scroll_offset) {
                state.scroll_offset = state.selected_index;
            }
        }
    }
}

void menu_down(void)
{
    if (!is_active) {
        return;
    }

    if (state.editing_value) {
        /* Decrease value */
        const menu_item_t *item = &state.current_menu[state.selected_index];
        if (edit_value >= (item->value_min + item->value_step)) {
            edit_value -= item->value_step;
        }
    } else {
        /* Navigate down */
        if (state.selected_index < (state.menu_count - 1U)) {
            state.selected_index++;
            if (state.selected_index >= (state.scroll_offset + VISIBLE_ITEMS)) {
                state.scroll_offset = state.selected_index - VISIBLE_ITEMS + 1U;
            }
        }
    }
}

void menu_select(void)
{
    if (!is_active || (state.current_menu == NULL)) {
        return;
    }

    const menu_item_t *item = &state.current_menu[state.selected_index];

    switch (item->type) {
    case MENU_TYPE_SUBMENU:
        if ((item->submenu != NULL) && (state.depth < (MENU_MAX_DEPTH - 1U))) {
            /* Save current state */
            state.menu_stack[state.depth] = state.current_menu;
            state.count_stack[state.depth] = state.menu_count;
            state.index_stack[state.depth] = state.selected_index;
            state.depth++;

            /* Enter submenu */
            state.current_menu = item->submenu;
            state.menu_count = item->submenu_count;
            state.selected_index = 0U;
            state.scroll_offset = 0U;
        }
        break;

    case MENU_TYPE_ACTION:
        if (item->action != NULL) {
            item->action();
        }
        break;

    case MENU_TYPE_VALUE:
        if (state.editing_value) {
            /* Confirm edit */
            if (g_settings != NULL) {
                /* Apply value based on menu item */
                if (strstr(item->name, "FTP") != NULL) {
                    user_settings_set_ftp(g_settings, edit_value);
                } else if (strstr(item->name, "Weight") != NULL) {
                    user_settings_set_weight(g_settings, edit_value);
                }
            }
            state.editing_value = false;
        } else {
            /* Start editing */
            if (g_settings != NULL) {
                if (strstr(item->name, "FTP") != NULL) {
                    edit_value = user_settings_get_ftp(g_settings);
                } else if (strstr(item->name, "Weight") != NULL) {
                    edit_value = user_settings_get_weight(g_settings);
                }
            }
            state.editing_value = true;
        }
        break;

    case MENU_TYPE_TOGGLE:
        /* Toggle value */
        if (item->value_ptr != NULL) {
            *item->value_ptr = (*item->value_ptr == 0U) ? 1U : 0U;
        }
        break;

    case MENU_TYPE_BACK:
        menu_back();
        break;

    default:
        break;
    }
}

void menu_back(void)
{
    if (!is_active) {
        return;
    }

    if (state.editing_value) {
        /* Cancel edit */
        state.editing_value = false;
    } else if (state.depth > 0U) {
        /* Return to parent menu */
        state.depth--;
        state.current_menu = state.menu_stack[state.depth];
        state.menu_count = state.count_stack[state.depth];
        state.selected_index = state.index_stack[state.depth];
        state.scroll_offset = 0U;
        if (state.selected_index >= VISIBLE_ITEMS) {
            state.scroll_offset = state.selected_index - VISIBLE_ITEMS + 1U;
        }
    } else {
        /* Close menu at root level */
        menu_close();
    }
}

const menu_state_t *menu_get_state(void)
{
    return &state;
}

bool menu_handle_button(btn_event_t event)
{
    if (!is_active) {
        return false;
    }

    switch (event) {
    case BTN_EVENT_LEFT:
        menu_up();
        return true;

    case BTN_EVENT_RIGHT:
        menu_down();
        return true;

    case BTN_EVENT_CENTER:
        menu_select();
        return true;

    case BTN_EVENT_LONG_LEFT:
    case BTN_EVENT_LONG_RIGHT:
        menu_back();
        return true;

    default:
        break;
    }

    return false;
}

uint16_t menu_get_edit_value(void)
{
    return edit_value;
}
