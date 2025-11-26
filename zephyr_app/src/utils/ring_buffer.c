/**
 * @file ring_buffer.c
 * @brief Generic ring buffer implementation
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "app_types.h"

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

typedef struct {
    uint8_t *buffer;
    size_t size;
    size_t head;
    size_t tail;
    size_t count;
} ring_buffer_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize ring buffer
 */
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, size_t size)
{
    if ((rb == NULL) || (buffer == NULL) || (size == 0U)) {
        return;
    }

    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0U;
    rb->tail = 0U;
    rb->count = 0U;
}

/**
 * @brief Put byte into ring buffer
 */
bool ring_buffer_put(ring_buffer_t *rb, uint8_t byte)
{
    if ((rb == NULL) || (rb->count >= rb->size)) {
        return false;
    }

    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1U) % rb->size;
    rb->count++;

    return true;
}

/**
 * @brief Get byte from ring buffer
 */
bool ring_buffer_get(ring_buffer_t *rb, uint8_t *byte)
{
    if ((rb == NULL) || (byte == NULL) || (rb->count == 0U)) {
        return false;
    }

    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1U) % rb->size;
    rb->count--;

    return true;
}

/**
 * @brief Check if buffer is empty
 */
bool ring_buffer_is_empty(const ring_buffer_t *rb)
{
    if (rb == NULL) {
        return true;
    }

    return (rb->count == 0U);
}

/**
 * @brief Check if buffer is full
 */
bool ring_buffer_is_full(const ring_buffer_t *rb)
{
    if (rb == NULL) {
        return false;
    }

    return (rb->count >= rb->size);
}

/**
 * @brief Get number of bytes in buffer
 */
size_t ring_buffer_count(const ring_buffer_t *rb)
{
    if (rb == NULL) {
        return 0U;
    }

    return rb->count;
}

/**
 * @brief Get free space in buffer
 */
size_t ring_buffer_free(const ring_buffer_t *rb)
{
    if (rb == NULL) {
        return 0U;
    }

    return rb->size - rb->count;
}

/**
 * @brief Clear buffer
 */
void ring_buffer_clear(ring_buffer_t *rb)
{
    if (rb == NULL) {
        return;
    }

    rb->head = 0U;
    rb->tail = 0U;
    rb->count = 0U;
}
