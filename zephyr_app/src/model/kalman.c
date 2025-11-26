/**
 * @file kalman.c
 * @brief Kalman filter implementation
 */

#include <string.h>
#include "model/kalman.h"

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void kalman_init(kalman_state_t *state, float process_noise, float measurement_noise)
{
    if (state == NULL) {
        return;
    }

    state->x = 0.0f;
    state->p = 1.0f;
    state->q = process_noise;
    state->r = measurement_noise;
    state->k = 0.0f;
    state->initialized = false;
}

float kalman_update(kalman_state_t *state, float measurement)
{
    if (state == NULL) {
        return 0.0f;
    }

    if (!state->initialized) {
        /* First measurement - initialize state */
        state->x = measurement;
        state->p = 1.0f;
        state->initialized = true;
        return state->x;
    }

    /* Prediction step */
    /* x_pred = x (no state transition model, assuming constant) */
    /* p_pred = p + q */
    state->p = state->p + state->q;

    /* Update step */
    /* k = p_pred / (p_pred + r) */
    state->k = state->p / (state->p + state->r);

    /* x = x_pred + k * (measurement - x_pred) */
    state->x = state->x + state->k * (measurement - state->x);

    /* p = (1 - k) * p_pred */
    state->p = (1.0f - state->k) * state->p;

    return state->x;
}

void kalman_set(kalman_state_t *state, float value)
{
    if (state == NULL) {
        return;
    }

    state->x = value;
    state->initialized = true;
}

float kalman_get(const kalman_state_t *state)
{
    if (state == NULL) {
        return 0.0f;
    }

    return state->x;
}

void kalman_reset(kalman_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->x = 0.0f;
    state->p = 1.0f;
    state->k = 0.0f;
    state->initialized = false;
}

void kalman_position_init(kalman_position_t *pos)
{
    if (pos == NULL) {
        return;
    }

    /* GPS position noise characteristics */
    /* Lower Q = smoother, slower response */
    /* Lower R = trusts measurements more */

    /* Latitude/Longitude: small Q, moderate R */
    kalman_init(&pos->lat, 0.00001f, 0.0001f);
    kalman_init(&pos->lon, 0.00001f, 0.0001f);

    /* Altitude: more noise expected */
    kalman_init(&pos->alt, 0.1f, 1.0f);

    /* Speed: moderate smoothing */
    kalman_init(&pos->speed, 0.5f, 2.0f);
}

loc_data_t kalman_position_update(kalman_position_t *pos, const loc_data_t *loc)
{
    loc_data_t filtered = {0};

    if ((pos == NULL) || (loc == NULL)) {
        return filtered;
    }

    filtered.lat = kalman_update(&pos->lat, loc->lat);
    filtered.lon = kalman_update(&pos->lon, loc->lon);
    filtered.alt = kalman_update(&pos->alt, loc->alt);
    filtered.speed = kalman_update(&pos->speed, loc->speed);
    filtered.course = loc->course;  /* Don't filter course */
    filtered.timestamp = loc->timestamp;

    return filtered;
}

void kalman_position_reset(kalman_position_t *pos)
{
    if (pos == NULL) {
        return;
    }

    kalman_reset(&pos->lat);
    kalman_reset(&pos->lon);
    kalman_reset(&pos->alt);
    kalman_reset(&pos->speed);
}
