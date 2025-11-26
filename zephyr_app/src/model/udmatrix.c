/**
 * @file udmatrix.c
 * @brief UD Matrix library implementation
 */

#include <string.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "model/udmatrix.h"

LOG_MODULE_REGISTER(udmatrix, CONFIG_LOG_DEFAULT_LEVEL);

/* ==========================================================================
 * Matrix Initialization
 * ========================================================================== */

void udmat_init(udmatrix_t *mat, uint8_t rows, uint8_t cols)
{
    if (mat == NULL) {
        return;
    }

    if (rows > UDMAT_MAX_DIM) {
        rows = UDMAT_MAX_DIM;
    }
    if (cols > UDMAT_MAX_DIM) {
        cols = UDMAT_MAX_DIM;
    }

    mat->rows = rows;
    mat->cols = cols;
    (void)memset(mat->data, 0, sizeof(mat->data));
}

void udmat_resize(udmatrix_t *mat, uint8_t rows, uint8_t cols)
{
    udmat_init(mat, rows, cols);
}

void udmat_zeros(udmatrix_t *mat)
{
    if (mat == NULL) {
        return;
    }

    (void)memset(mat->data, 0, sizeof(mat->data));
}

void udmat_identity(udmatrix_t *mat, float factor)
{
    if (mat == NULL) {
        return;
    }

    udmat_zeros(mat);

    uint8_t min_dim = (mat->rows < mat->cols) ? mat->rows : mat->cols;
    for (uint8_t i = 0U; i < min_dim; i++) {
        mat->data[i][i] = factor;
    }
}

void udmat_ones(udmatrix_t *mat, float factor)
{
    udmat_identity(mat, factor);
}

/* ==========================================================================
 * Element Access
 * ========================================================================== */

float udmat_get(const udmatrix_t *mat, uint8_t row, uint8_t col)
{
    if ((mat == NULL) || (row >= mat->rows) || (col >= mat->cols)) {
        return 0.0f;
    }

    return mat->data[row][col];
}

void udmat_set(udmatrix_t *mat, uint8_t row, uint8_t col, float value)
{
    if ((mat == NULL) || (row >= mat->rows) || (col >= mat->cols)) {
        return;
    }

    mat->data[row][col] = value;
}

/* ==========================================================================
 * Matrix Operations
 * ========================================================================== */

bool udmat_add(const udmatrix_t *a, const udmatrix_t *b, udmatrix_t *result)
{
    if ((a == NULL) || (b == NULL) || (result == NULL)) {
        return false;
    }

    if ((a->rows != b->rows) || (a->cols != b->cols)) {
        return false;
    }

    udmat_init(result, a->rows, a->cols);

    for (uint8_t i = 0U; i < a->rows; i++) {
        for (uint8_t j = 0U; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }

    return true;
}

bool udmat_sub(const udmatrix_t *a, const udmatrix_t *b, udmatrix_t *result)
{
    if ((a == NULL) || (b == NULL) || (result == NULL)) {
        return false;
    }

    if ((a->rows != b->rows) || (a->cols != b->cols)) {
        return false;
    }

    udmat_init(result, a->rows, a->cols);

    for (uint8_t i = 0U; i < a->rows; i++) {
        for (uint8_t j = 0U; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] - b->data[i][j];
        }
    }

    return true;
}

bool udmat_mul(const udmatrix_t *a, const udmatrix_t *b, udmatrix_t *result)
{
    if ((a == NULL) || (b == NULL) || (result == NULL)) {
        return false;
    }

    if (a->cols != b->rows) {
        return false;
    }

    /* Create temp to allow result to be same as a or b */
    udmatrix_t temp;
    udmat_init(&temp, a->rows, b->cols);

    for (uint8_t i = 0U; i < a->rows; i++) {
        for (uint8_t j = 0U; j < b->cols; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < a->cols; k++) {
                sum += a->data[i][k] * b->data[k][j];
            }
            temp.data[i][j] = sum;
        }
    }

    udmat_copy(&temp, result);
    return true;
}

void udmat_transpose(const udmatrix_t *a, udmatrix_t *result)
{
    if ((a == NULL) || (result == NULL)) {
        return;
    }

    udmatrix_t temp;
    udmat_init(&temp, a->cols, a->rows);

    for (uint8_t i = 0U; i < a->rows; i++) {
        for (uint8_t j = 0U; j < a->cols; j++) {
            temp.data[j][i] = a->data[i][j];
        }
    }

    udmat_copy(&temp, result);
}

bool udmat_invert(const udmatrix_t *a, udmatrix_t *result)
{
    if ((a == NULL) || (result == NULL)) {
        return false;
    }

    if (a->rows != a->cols) {
        return false;  /* Not square */
    }

    uint8_t n = a->rows;

    /* Gauss-Jordan elimination for small matrices */
    /* Create augmented matrix [A | I] */
    float aug[UDMAT_MAX_DIM][UDMAT_MAX_DIM * 2U];
    (void)memset(aug, 0, sizeof(aug));

    /* Copy A to left side */
    for (uint8_t i = 0U; i < n; i++) {
        for (uint8_t j = 0U; j < n; j++) {
            aug[i][j] = a->data[i][j];
        }
        /* Identity on right side */
        aug[i][n + i] = 1.0f;
    }

    /* Forward elimination with partial pivoting */
    for (uint8_t col = 0U; col < n; col++) {
        /* Find pivot */
        float max_val = fabsf(aug[col][col]);
        uint8_t max_row = col;

        for (uint8_t row = col + 1U; row < n; row++) {
            if (fabsf(aug[row][col]) > max_val) {
                max_val = fabsf(aug[row][col]);
                max_row = row;
            }
        }

        /* Check for singular matrix */
        if (max_val < 1e-10f) {
            return false;
        }

        /* Swap rows if needed */
        if (max_row != col) {
            for (uint8_t j = 0U; j < (2U * n); j++) {
                float temp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = temp;
            }
        }

        /* Scale pivot row */
        float pivot = aug[col][col];
        for (uint8_t j = 0U; j < (2U * n); j++) {
            aug[col][j] /= pivot;
        }

        /* Eliminate column */
        for (uint8_t row = 0U; row < n; row++) {
            if (row != col) {
                float factor = aug[row][col];
                for (uint8_t j = 0U; j < (2U * n); j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }
    }

    /* Extract inverse from right side */
    udmat_init(result, n, n);
    for (uint8_t i = 0U; i < n; i++) {
        for (uint8_t j = 0U; j < n; j++) {
            result->data[i][j] = aug[i][n + j];
        }
    }

    return true;
}

void udmat_bound(udmatrix_t *mat, float min, float max)
{
    if (mat == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < mat->rows; i++) {
        for (uint8_t j = 0U; j < mat->cols; j++) {
            if (mat->data[i][j] < min) {
                mat->data[i][j] = min;
            } else if (mat->data[i][j] > max) {
                mat->data[i][j] = max;
            } else {
                /* Value is within bounds */
            }
        }
    }
}

void udmat_copy(const udmatrix_t *src, udmatrix_t *dst)
{
    if ((src == NULL) || (dst == NULL)) {
        return;
    }

    dst->rows = src->rows;
    dst->cols = src->cols;
    (void)memcpy(dst->data, src->data, sizeof(dst->data));
}

bool udmat_is_empty(const udmatrix_t *mat)
{
    if (mat == NULL) {
        return true;
    }

    return ((mat->rows == 0U) || (mat->cols == 0U));
}

/* ==========================================================================
 * Vector Operations
 * ========================================================================== */

void udvec_init(udvector_t *vec, uint8_t size)
{
    if (vec == NULL) {
        return;
    }

    if (size > UDMAT_MAX_DIM) {
        size = UDMAT_MAX_DIM;
    }

    vec->size = size;
    (void)memset(vec->data, 0, sizeof(vec->data));
}

void udvec_zeros(udvector_t *vec)
{
    if (vec == NULL) {
        return;
    }

    (void)memset(vec->data, 0, sizeof(vec->data));
}

float udvec_get(const udvector_t *vec, uint8_t idx)
{
    if ((vec == NULL) || (idx >= vec->size)) {
        return 0.0f;
    }

    return vec->data[idx];
}

void udvec_set(udvector_t *vec, uint8_t idx, float value)
{
    if ((vec == NULL) || (idx >= vec->size)) {
        return;
    }

    vec->data[idx] = value;
}

/* ==========================================================================
 * Debug
 * ========================================================================== */

void udmat_print(const udmatrix_t *mat, const char *name)
{
    if (mat == NULL) {
        return;
    }

    LOG_DBG("Matrix %s [%dx%d]:", (name != NULL) ? name : "?", mat->rows, mat->cols);

    for (uint8_t i = 0U; i < mat->rows; i++) {
        for (uint8_t j = 0U; j < mat->cols; j++) {
            LOG_DBG("  [%d,%d] = %.4f", i, j, (double)mat->data[i][j]);
        }
    }
}
