/**
 * @file crash_recovery.h
 * @brief Crash recovery and fault tracking system
 * @note Follows MISRA C:2012 guidelines
 *
 * Provides crash detection, fault logging, and data recovery after
 * unexpected resets. Uses retained RAM section to persist data
 * across warm resets.
 */

#ifndef MODEL_CRASH_RECOVERY_H_
#define MODEL_CRASH_RECOVERY_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Magic number to identify valid crash data */
#define CRASH_MAGIC             0xDEADBEEFU

/** CRC position marker (from original) */
#define CRASH_DESCR_POS_CRC     0xDBU

/** Maximum error description buffer size */
#define CRASH_ERR_BUFFER_SIZE   210U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Hard fault stack contents
 */
typedef struct {
    uint32_t r0;        /**< R0 register */
    uint32_t r1;        /**< R1 register */
    uint32_t r2;        /**< R2 register */
    uint32_t r3;        /**< R3 register */
    uint32_t r12;       /**< R12 register */
    uint32_t lr;        /**< Link register */
    uint32_t pc;        /**< Program counter */
    uint32_t psr;       /**< Program status register */
} hardfault_stack_t;

/**
 * @brief Hard fault description with CRC
 */
typedef struct {
    hardfault_stack_t stack;    /**< CPU registers at fault */
    uint8_t crc;                /**< CRC-8 checksum */
} hardfault_desc_t;

/**
 * @brief Error description structure
 */
typedef struct {
    char buffer[CRASH_ERR_BUFFER_SIZE]; /**< Error message buffer */
    uint32_t pc;                        /**< Program counter at error */
    uint32_t error_id;                  /**< Error identifier */
    uint8_t crc;                        /**< CRC-8 checksum */
} error_desc_t;

/**
 * @brief Saved attitude data with CRC
 */
typedef struct {
    loc_data_t loc;         /**< Last known location */
    date_data_t date;       /**< Last known date/time */
    float dist;             /**< Total distance */
    float climb;            /**< Total climb */
    uint16_t nbpts;         /**< Number of GPS points */
    uint16_t nbsec_act;     /**< Active seconds */
    uint8_t crc;            /**< CRC-8 checksum */
} saved_data_t;

/**
 * @brief Crash recovery descriptor (stored in retained RAM)
 */
typedef struct {
    uint32_t magic;             /**< Magic number for validation */
    uint32_t reset_count;       /**< Number of resets detected */
    uint32_t task_id;           /**< Task ID at time of crash */
    error_desc_t error;         /**< Error description */
    saved_data_t saved;         /**< Saved application data */
    hardfault_desc_t hardfault; /**< Hard fault info */
    uint8_t fault_type;         /**< Type of fault (0=none, 1=hardfault, 2=error) */
} crash_descriptor_t;

/**
 * @brief Reset reason enumeration
 */
typedef enum {
    RESET_REASON_UNKNOWN = 0,
    RESET_REASON_POWER_ON,      /**< Power-on reset */
    RESET_REASON_PIN,           /**< External pin reset */
    RESET_REASON_WATCHDOG,      /**< Watchdog timeout */
    RESET_REASON_SOFTWARE,      /**< Software reset */
    RESET_REASON_LOCKUP,        /**< CPU lockup */
    RESET_REASON_BROWNOUT,      /**< Brownout detection */
    RESET_REASON_HARDFAULT,     /**< Hard fault handler reset */
} reset_reason_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize crash recovery system
 * @return APP_OK on success
 */
app_err_t crash_recovery_init(void);

/**
 * @brief Check if crash data is present from previous session
 * @return true if valid crash data exists
 */
bool crash_recovery_has_data(void);

/**
 * @brief Get crash descriptor
 * @return Pointer to crash descriptor or NULL
 */
const crash_descriptor_t *crash_recovery_get_descriptor(void);

/**
 * @brief Get last reset reason
 * @return Reset reason enumeration
 */
reset_reason_t crash_recovery_get_reset_reason(void);

/**
 * @brief Get reset count
 * @return Number of resets since last power-on
 */
uint32_t crash_recovery_get_reset_count(void);

/**
 * @brief Save current application state for recovery
 * @param loc Current location
 * @param date Current date/time
 * @param dist Total distance
 * @param climb Total climb
 * @param nbpts Number of GPS points
 * @param nbsec_act Active seconds
 */
void crash_recovery_save_state(const loc_data_t *loc,
                               const date_data_t *date,
                               float dist,
                               float climb,
                               uint16_t nbpts,
                               uint16_t nbsec_act);

/**
 * @brief Get saved state if available
 * @param data Pointer to saved data structure to fill
 * @return true if valid saved data was recovered
 */
bool crash_recovery_get_saved_state(saved_data_t *data);

/**
 * @brief Log an error for crash tracking
 * @param error_id Error identifier
 * @param pc Program counter (use __builtin_return_address(0))
 * @param format Printf format string
 * @param ... Format arguments
 */
void crash_recovery_log_error(uint32_t error_id, uint32_t pc,
                              const char *format, ...);

/**
 * @brief Clear crash recovery data
 */
void crash_recovery_clear(void);

/**
 * @brief Hard fault handler hook
 * @param stack Pointer to stack at time of fault
 *
 * Call this from the hard fault handler to save fault information
 */
void crash_recovery_hardfault_handler(const hardfault_stack_t *stack);

/**
 * @brief Print crash recovery information to log
 */
void crash_recovery_print_info(void);

/**
 * @brief Set current task ID for tracking
 * @param task_id Task identifier
 */
void crash_recovery_set_task_id(uint32_t task_id);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_CRASH_RECOVERY_H_ */
