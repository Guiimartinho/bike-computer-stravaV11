/**
 * @file menu.h
 * @brief Menu system for user settings and configuration
 *
 * Follows MISRA C:2012 guidelines.
 */

#ifndef VUE_MENU_H
#define VUE_MENU_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Constants
 * ========================================================================== */

/** Maximum menu items per page */
#define MENU_MAX_ITEMS      8U

/** Maximum menu depth (nested menus) */
#define MENU_MAX_DEPTH      4U

/** Menu item name max length */
#define MENU_NAME_MAX       20U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Menu item types */
typedef enum {
    MENU_TYPE_SUBMENU = 0,  /**< Opens a submenu */
    MENU_TYPE_ACTION,       /**< Executes an action */
    MENU_TYPE_VALUE,        /**< Editable value */
    MENU_TYPE_TOGGLE,       /**< On/Off toggle */
    MENU_TYPE_BACK          /**< Go back to parent */
} menu_type_t;

/** Menu item structure */
typedef struct menu_item {
    char name[MENU_NAME_MAX];       /**< Display name */
    menu_type_t type;               /**< Item type */
    void (*action)(void);           /**< Action callback (for ACTION type) */
    uint16_t *value_ptr;            /**< Pointer to value (for VALUE type) */
    uint16_t value_min;             /**< Minimum value */
    uint16_t value_max;             /**< Maximum value */
    uint16_t value_step;            /**< Value step */
    const struct menu_item *submenu;/**< Submenu pointer (for SUBMENU type) */
    uint8_t submenu_count;          /**< Number of items in submenu */
} menu_item_t;

/** Menu state */
typedef struct {
    const menu_item_t *current_menu;    /**< Current menu pointer */
    uint8_t menu_count;                 /**< Items in current menu */
    uint8_t selected_index;             /**< Currently selected item */
    uint8_t scroll_offset;              /**< Scroll offset for long menus */
    bool editing_value;                 /**< True if editing a value */
    uint8_t depth;                      /**< Current menu depth */
    const menu_item_t *menu_stack[MENU_MAX_DEPTH];  /**< Menu stack for navigation */
    uint8_t count_stack[MENU_MAX_DEPTH];            /**< Count stack */
    uint8_t index_stack[MENU_MAX_DEPTH];            /**< Index stack */
} menu_state_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize menu system
 * @return APP_OK on success
 */
app_err_t menu_init(void);

/**
 * @brief Check if menu is active
 * @return true if menu is currently displayed
 */
bool menu_is_active(void);

/**
 * @brief Open the main menu
 */
void menu_open(void);

/**
 * @brief Close menu and return to normal view
 */
void menu_close(void);

/**
 * @brief Navigate up in menu
 */
void menu_up(void);

/**
 * @brief Navigate down in menu
 */
void menu_down(void);

/**
 * @brief Select current item / confirm edit
 */
void menu_select(void);

/**
 * @brief Go back to parent menu
 */
void menu_back(void);

/**
 * @brief Get current menu state for rendering
 * @return Pointer to menu state
 */
const menu_state_t *menu_get_state(void);

/**
 * @brief Handle button event in menu
 * @param event Button event
 * @return true if event was handled
 */
bool menu_handle_button(btn_event_t event);

/**
 * @brief Get current edit value (when editing)
 * @return Current edit value
 */
uint16_t menu_get_edit_value(void);

#ifdef __cplusplus
}
#endif

#endif /* VUE_MENU_H */
