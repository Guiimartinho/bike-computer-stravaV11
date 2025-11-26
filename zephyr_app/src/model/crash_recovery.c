/**
 * @file crash_recovery.c
 * @brief Crash recovery and fault tracking implementation
 *
 * Uses Zephyr retained RAM section to persist crash data across
 * warm resets. Integrates with nRF reset reason registers.
 */

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#if defined(CONFIG_SOC_SERIES_NRF52X)
#include <hal/nrf_power.h>
#endif

#include "model/crash_recovery.h"

LOG_MODULE_REGISTER(crash_recovery, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Retained RAM Section
 * ========================================================================== */

/**
 * Crash descriptor stored in retained RAM (no-init section)
 * This data persists across warm resets but not power cycles
 */
static __noinit crash_descriptor_t g_crash_desc;

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** CFSR register bit definitions for Cortex-M4 */
#define CFSR_MMARVALID      (1U << 7U)
#define CFSR_BFARVALID      (1U << 15U)

/** Error cause messages for CFSR bits */
static const char *cfsr_messages[] = {
    [0]  = "Undefined instruction",
    [1]  = "Invalid load/store location",
    [2]  = NULL,
    [3]  = "Unstack access violation",
    [4]  = "Stack access violation",
    [5]  = "FP lazy state MemManage fault",
    [6]  = NULL,
    [7]  = NULL,
    [8]  = "Instruction bus error",
    [9]  = "Data bus error (precise)",
    [10] = "Data bus error (imprecise)",
    [11] = "Unstack BusFault",
    [12] = "Stack BusFault",
    [13] = "FP lazy state BusFault",
    [14] = NULL,
    [15] = NULL,
    [16] = "Undefined instruction",
    [17] = "Illegal EPSR use",
    [18] = "Invalid EXC_RETURN",
    [19] = "Coprocessor access",
    [20] = NULL,
    [21] = NULL,
    [22] = NULL,
    [23] = NULL,
    [24] = "Unaligned memory access",
    [25] = "Division by zero",
};

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate CRC-8 of data array
 */
static uint8_t calculate_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0U;

    for (size_t i = 0U; i < len; i++) {
        uint8_t inbyte = data[i];

        for (uint8_t j = 0U; j < 8U; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01U;
            crc >>= 1U;

            if (mix != 0U) {
                crc ^= 0x8CU;
            }

            inbyte >>= 1U;
        }
    }

    return crc;
}

/**
 * @brief Update saved data CRC
 */
static void update_saved_crc(saved_data_t *data)
{
    data->crc = calculate_crc8((const uint8_t *)data,
                               sizeof(saved_data_t) - sizeof(uint8_t));
}

/**
 * @brief Verify saved data CRC
 */
static bool verify_saved_crc(const saved_data_t *data)
{
    uint8_t calc = calculate_crc8((const uint8_t *)data,
                                  sizeof(saved_data_t) - sizeof(uint8_t));
    return (calc == data->crc);
}

/**
 * @brief Update hardfault CRC
 */
static void update_hardfault_crc(hardfault_desc_t *desc)
{
    desc->crc = calculate_crc8((const uint8_t *)&desc->stack,
                               sizeof(hardfault_stack_t));
}

/**
 * @brief Verify hardfault CRC
 */
static bool verify_hardfault_crc(const hardfault_desc_t *desc)
{
    uint8_t calc = calculate_crc8((const uint8_t *)&desc->stack,
                                  sizeof(hardfault_stack_t));
    return (calc == desc->crc);
}

/**
 * @brief Read reset reason from hardware
 */
static reset_reason_t read_hw_reset_reason(void)
{
#if defined(CONFIG_SOC_SERIES_NRF52X)
    uint32_t reason = nrf_power_resetreas_get(NRF_POWER);

    /* Clear the reset reason register */
    nrf_power_resetreas_clear(NRF_POWER, reason);

    if (reason & NRF_POWER_RESETREAS_RESETPIN_MASK) {
        return RESET_REASON_PIN;
    }
    if (reason & NRF_POWER_RESETREAS_DOG_MASK) {
        return RESET_REASON_WATCHDOG;
    }
    if (reason & NRF_POWER_RESETREAS_SREQ_MASK) {
        return RESET_REASON_SOFTWARE;
    }
    if (reason & NRF_POWER_RESETREAS_LOCKUP_MASK) {
        return RESET_REASON_LOCKUP;
    }
    if (reason == 0U) {
        return RESET_REASON_POWER_ON;
    }

    return RESET_REASON_UNKNOWN;
#else
    return RESET_REASON_UNKNOWN;
#endif
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

app_err_t crash_recovery_init(void)
{
    reset_reason_t reason = read_hw_reset_reason();

    /* Check if we have valid crash data from previous session */
    if (g_crash_desc.magic == CRASH_MAGIC) {
        /* Valid crash data exists - increment reset count */
        g_crash_desc.reset_count++;

        LOG_WRN("Crash data detected from previous session");
        LOG_WRN("Reset count: %u", g_crash_desc.reset_count);

        if (g_crash_desc.fault_type == 1U) {
            LOG_ERR("Previous session ended with HARDFAULT");
        } else if (g_crash_desc.fault_type == 2U) {
            LOG_ERR("Previous session ended with ERROR");
        }
    } else {
        /* No valid crash data - initialize */
        (void)memset(&g_crash_desc, 0, sizeof(crash_descriptor_t));
        g_crash_desc.magic = CRASH_MAGIC;
        g_crash_desc.reset_count = 0U;
        g_crash_desc.fault_type = 0U;

        LOG_INF("Crash recovery initialized (fresh start)");
    }

    LOG_INF("Reset reason: %d", (int)reason);

    return APP_OK;
}

bool crash_recovery_has_data(void)
{
    if (g_crash_desc.magic != CRASH_MAGIC) {
        return false;
    }

    return (g_crash_desc.fault_type != 0U);
}

const crash_descriptor_t *crash_recovery_get_descriptor(void)
{
    if (g_crash_desc.magic != CRASH_MAGIC) {
        return NULL;
    }

    return &g_crash_desc;
}

reset_reason_t crash_recovery_get_reset_reason(void)
{
    return read_hw_reset_reason();
}

uint32_t crash_recovery_get_reset_count(void)
{
    if (g_crash_desc.magic != CRASH_MAGIC) {
        return 0U;
    }

    return g_crash_desc.reset_count;
}

void crash_recovery_save_state(const loc_data_t *loc,
                               const date_data_t *date,
                               float dist,
                               float climb,
                               uint16_t nbpts,
                               uint16_t nbsec_act)
{
    if (g_crash_desc.magic != CRASH_MAGIC) {
        return;
    }

    if (loc != NULL) {
        (void)memcpy(&g_crash_desc.saved.loc, loc, sizeof(loc_data_t));
    }

    if (date != NULL) {
        (void)memcpy(&g_crash_desc.saved.date, date, sizeof(date_data_t));
    }

    g_crash_desc.saved.dist = dist;
    g_crash_desc.saved.climb = climb;
    g_crash_desc.saved.nbpts = nbpts;
    g_crash_desc.saved.nbsec_act = nbsec_act;

    update_saved_crc(&g_crash_desc.saved);
}

bool crash_recovery_get_saved_state(saved_data_t *data)
{
    if (data == NULL) {
        return false;
    }

    if (g_crash_desc.magic != CRASH_MAGIC) {
        return false;
    }

    if (!verify_saved_crc(&g_crash_desc.saved)) {
        LOG_WRN("Saved state CRC invalid");
        return false;
    }

    (void)memcpy(data, &g_crash_desc.saved, sizeof(saved_data_t));
    return true;
}

void crash_recovery_log_error(uint32_t error_id, uint32_t pc,
                              const char *format, ...)
{
    if (g_crash_desc.magic != CRASH_MAGIC) {
        return;
    }

    g_crash_desc.fault_type = 2U; /* Error type */
    g_crash_desc.error.error_id = error_id;
    g_crash_desc.error.pc = pc;

    /* Format error message */
    va_list args;
    va_start(args, format);
    (void)vsnprintf(g_crash_desc.error.buffer,
                    sizeof(g_crash_desc.error.buffer),
                    format, args);
    va_end(args);

    /* Calculate CRC */
    g_crash_desc.error.crc = calculate_crc8(
        (const uint8_t *)&g_crash_desc.error,
        sizeof(error_desc_t) - sizeof(uint8_t)
    );

    LOG_ERR("Error logged: ID=%u PC=0x%08X: %s",
            error_id, pc, g_crash_desc.error.buffer);
}

void crash_recovery_clear(void)
{
    g_crash_desc.fault_type = 0U;
    g_crash_desc.task_id = 0U;
    (void)memset(&g_crash_desc.error, 0, sizeof(error_desc_t));
    (void)memset(&g_crash_desc.hardfault, 0, sizeof(hardfault_desc_t));

    LOG_INF("Crash recovery data cleared");
}

void crash_recovery_hardfault_handler(const hardfault_stack_t *stack)
{
    if (stack == NULL) {
        return;
    }

    /* Mark as hardfault */
    g_crash_desc.magic = CRASH_MAGIC;
    g_crash_desc.fault_type = 1U;

    /* Copy stack contents */
    (void)memcpy(&g_crash_desc.hardfault.stack, stack, sizeof(hardfault_stack_t));

    /* Update CRC */
    update_hardfault_crc(&g_crash_desc.hardfault);

    /* Log fault info to buffer for later retrieval */
#if (__CORTEX_M == 4)
    uint32_t cfsr = SCB->CFSR;

    /* Find cause */
    for (size_t i = 0U; i < ARRAY_SIZE(cfsr_messages); i++) {
        if (((cfsr & (1U << i)) != 0U) && (cfsr_messages[i] != NULL)) {
            (void)snprintf(g_crash_desc.error.buffer,
                           sizeof(g_crash_desc.error.buffer),
                           "HardFault: %s at PC=0x%08X",
                           cfsr_messages[i], stack->pc);
            break;
        }
    }

    g_crash_desc.error.pc = stack->pc;
#endif
}

void crash_recovery_print_info(void)
{
    if (g_crash_desc.magic != CRASH_MAGIC) {
        LOG_INF("No crash data available");
        return;
    }

    LOG_INF("=== Crash Recovery Info ===");
    LOG_INF("Reset count: %u", g_crash_desc.reset_count);
    LOG_INF("Fault type: %u", g_crash_desc.fault_type);
    LOG_INF("Task ID: %u", g_crash_desc.task_id);

    if (g_crash_desc.fault_type == 1U) {
        /* Hard fault */
        if (verify_hardfault_crc(&g_crash_desc.hardfault)) {
            LOG_ERR("=== Hard Fault Info ===");
            LOG_ERR("PC:  0x%08X", g_crash_desc.hardfault.stack.pc);
            LOG_ERR("LR:  0x%08X", g_crash_desc.hardfault.stack.lr);
            LOG_ERR("PSR: 0x%08X", g_crash_desc.hardfault.stack.psr);
            LOG_ERR("R0:  0x%08X  R1:  0x%08X",
                    g_crash_desc.hardfault.stack.r0,
                    g_crash_desc.hardfault.stack.r1);
            LOG_ERR("R2:  0x%08X  R3:  0x%08X",
                    g_crash_desc.hardfault.stack.r2,
                    g_crash_desc.hardfault.stack.r3);
            LOG_ERR("R12: 0x%08X", g_crash_desc.hardfault.stack.r12);
        } else {
            LOG_WRN("Hard fault CRC invalid");
        }
    }

    if (g_crash_desc.fault_type == 2U) {
        /* Error */
        LOG_ERR("=== Error Info ===");
        LOG_ERR("Error ID: %u", g_crash_desc.error.error_id);
        LOG_ERR("PC: 0x%08X", g_crash_desc.error.pc);
        LOG_ERR("Message: %s", g_crash_desc.error.buffer);
    }

    if (verify_saved_crc(&g_crash_desc.saved)) {
        LOG_INF("=== Saved State ===");
        LOG_INF("Distance: %.1f m", (double)g_crash_desc.saved.dist);
        LOG_INF("Climb: %.1f m", (double)g_crash_desc.saved.climb);
        LOG_INF("GPS points: %u", g_crash_desc.saved.nbpts);
        LOG_INF("Active seconds: %u", g_crash_desc.saved.nbsec_act);
    }

    LOG_INF("===========================");
}

void crash_recovery_set_task_id(uint32_t task_id)
{
    if (g_crash_desc.magic == CRASH_MAGIC) {
        g_crash_desc.task_id = task_id;
    }
}
