/**
 * @file boucle.h
 * @brief Main application loop controller
 *
 * Manages the main processing loop and mode transitions.
 * Follows MISRA C:2012 guidelines.
 */

#ifndef MODEL_BOUCLE_H
#define MODEL_BOUCLE_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/** Boucle state */
typedef enum {
    BOUCLE_STATE_IDLE = 0,      /**< Idle/stopped */
    BOUCLE_STATE_RUNNING,       /**< Active recording */
    BOUCLE_STATE_PAUSED,        /**< Paused */
    BOUCLE_STATE_SAVING         /**< Saving data */
} boucle_state_t;

/** Boucle statistics */
typedef struct {
    uint32_t start_time;        /**< Start timestamp */
    uint32_t elapsed_time;      /**< Total elapsed time (s) */
    uint32_t moving_time;       /**< Time while moving (s) */
    float total_distance;       /**< Total distance (m) */
    float total_climb;          /**< Total elevation gain (m) */
    float max_speed;            /**< Maximum speed (km/h) */
    float avg_speed;            /**< Average speed (km/h) */
    uint16_t gps_points;        /**< Number of GPS points */
    uint8_t active_segments;    /**< Currently active segments */
    uint8_t pr_count;           /**< Personal record count */
} boucle_stats_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize boucle module
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_init(void);

/**
 * @brief Start activity recording
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_start(void);

/**
 * @brief Stop activity recording
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_stop(void);

/**
 * @brief Pause activity recording
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_pause(void);

/**
 * @brief Resume activity recording
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_resume(void);

/**
 * @brief Get current boucle state
 * @return Current state
 */
boucle_state_t boucle_get_state(void);

/**
 * @brief Get current statistics
 * @param stats Pointer to store statistics
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_get_stats(boucle_stats_t *stats);

/**
 * @brief Process one iteration of the main loop
 *
 * This should be called periodically (e.g., every 100ms).
 * It handles sensor polling, segment updates, and logging.
 */
void boucle_process(void);

/**
 * @brief Handle button event
 * @param event Button event
 */
void boucle_handle_button(btn_event_t event);

/**
 * @brief Set application mode
 * @param mode New application mode
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_set_mode(app_mode_t mode);

/**
 * @brief Get current application mode
 * @return Current mode
 */
app_mode_t boucle_get_mode(void);

/**
 * @brief Reset all statistics
 */
void boucle_reset_stats(void);

/**
 * @brief Save current activity to file
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_save_activity(void);

/**
 * @brief Check if activity is in progress
 * @return true if recording is active
 */
bool boucle_is_active(void);

/**
 * @brief Get current attitude data
 * @param att Pointer to store attitude
 * @return APP_OK on success, error code otherwise
 */
app_err_t boucle_get_attitude(attitude_t *att);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_BOUCLE_H */
