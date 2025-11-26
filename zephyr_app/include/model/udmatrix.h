/**
 * @file udmatrix.h
 * @brief UD Matrix library for Kalman filter operations
 * @note Follows MISRA C:2012 guidelines
 */

#ifndef MODEL_UDMATRIX_H_
#define MODEL_UDMATRIX_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum matrix dimension supported */
#define UDMAT_MAX_DIM       6U

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Matrix structure for UD decomposition Kalman filter
 */
typedef struct {
    float data[UDMAT_MAX_DIM][UDMAT_MAX_DIM];
    uint8_t rows;
    uint8_t cols;
} udmatrix_t;

/**
 * @brief Vector structure (column matrix)
 */
typedef struct {
    float data[UDMAT_MAX_DIM];
    uint8_t size;
} udvector_t;

/* ==========================================================================
 * Matrix Initialization
 * ========================================================================== */

/**
 * @brief Initialize matrix with zeros
 * @param mat Pointer to matrix
 * @param rows Number of rows
 * @param cols Number of columns
 */
void udmat_init(udmatrix_t *mat, uint8_t rows, uint8_t cols);

/**
 * @brief Resize matrix (clears data)
 * @param mat Pointer to matrix
 * @param rows New number of rows
 * @param cols New number of columns
 */
void udmat_resize(udmatrix_t *mat, uint8_t rows, uint8_t cols);

/**
 * @brief Set matrix to zeros
 * @param mat Pointer to matrix
 */
void udmat_zeros(udmatrix_t *mat);

/**
 * @brief Set matrix to identity (scaled by factor)
 * @param mat Pointer to matrix
 * @param factor Scale factor (1.0 for true identity)
 */
void udmat_identity(udmatrix_t *mat, float factor);

/**
 * @brief Set matrix diagonal to given values
 * @param mat Pointer to matrix
 * @param factor Factor to set diagonal elements
 */
void udmat_ones(udmatrix_t *mat, float factor);

/* ==========================================================================
 * Element Access
 * ========================================================================== */

/**
 * @brief Get matrix element
 * @param mat Pointer to matrix
 * @param row Row index
 * @param col Column index
 * @return Element value or 0.0 if out of bounds
 */
float udmat_get(const udmatrix_t *mat, uint8_t row, uint8_t col);

/**
 * @brief Set matrix element
 * @param mat Pointer to matrix
 * @param row Row index
 * @param col Column index
 * @param value Value to set
 */
void udmat_set(udmatrix_t *mat, uint8_t row, uint8_t col, float value);

/* ==========================================================================
 * Matrix Operations
 * ========================================================================== */

/**
 * @brief Matrix addition: result = a + b
 * @param a First matrix
 * @param b Second matrix
 * @param result Result matrix
 * @return true if success, false if dimension mismatch
 */
bool udmat_add(const udmatrix_t *a, const udmatrix_t *b, udmatrix_t *result);

/**
 * @brief Matrix subtraction: result = a - b
 * @param a First matrix
 * @param b Second matrix
 * @param result Result matrix
 * @return true if success, false if dimension mismatch
 */
bool udmat_sub(const udmatrix_t *a, const udmatrix_t *b, udmatrix_t *result);

/**
 * @brief Matrix multiplication: result = a * b
 * @param a First matrix (m x n)
 * @param b Second matrix (n x p)
 * @param result Result matrix (m x p)
 * @return true if success, false if dimension mismatch
 */
bool udmat_mul(const udmatrix_t *a, const udmatrix_t *b, udmatrix_t *result);

/**
 * @brief Matrix transpose: result = a^T
 * @param a Input matrix
 * @param result Result matrix
 */
void udmat_transpose(const udmatrix_t *a, udmatrix_t *result);

/**
 * @brief Matrix inverse (for small matrices)
 * @param a Input matrix
 * @param result Inverse matrix
 * @return true if invertible, false if singular
 */
bool udmat_invert(const udmatrix_t *a, udmatrix_t *result);

/**
 * @brief Bound matrix elements to range [min, max]
 * @param mat Pointer to matrix
 * @param min Minimum value
 * @param max Maximum value
 */
void udmat_bound(udmatrix_t *mat, float min, float max);

/**
 * @brief Copy matrix
 * @param src Source matrix
 * @param dst Destination matrix
 */
void udmat_copy(const udmatrix_t *src, udmatrix_t *dst);

/**
 * @brief Check if matrix is empty (0x0)
 * @param mat Pointer to matrix
 * @return true if empty
 */
bool udmat_is_empty(const udmatrix_t *mat);

/* ==========================================================================
 * Vector Operations
 * ========================================================================== */

/**
 * @brief Initialize vector with zeros
 * @param vec Pointer to vector
 * @param size Vector size
 */
void udvec_init(udvector_t *vec, uint8_t size);

/**
 * @brief Set vector to zeros
 * @param vec Pointer to vector
 */
void udvec_zeros(udvector_t *vec);

/**
 * @brief Get vector element
 * @param vec Pointer to vector
 * @param idx Index
 * @return Element value
 */
float udvec_get(const udvector_t *vec, uint8_t idx);

/**
 * @brief Set vector element
 * @param vec Pointer to vector
 * @param idx Index
 * @param value Value to set
 */
void udvec_set(udvector_t *vec, uint8_t idx, float value);

/* ==========================================================================
 * Debug
 * ========================================================================== */

/**
 * @brief Print matrix to debug log
 * @param mat Pointer to matrix
 * @param name Matrix name for logging
 */
void udmat_print(const udmatrix_t *mat, const char *name);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_UDMATRIX_H_ */
