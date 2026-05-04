/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * matrix_utils.c - Matrix mathematics utilities implementation for SynchroSpark
 * 
 * Copyright (C) 2025-2026 Ahmet Sekercioglu and Ismet Atalar
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "matrix_utils.h" 

static const char *TAG = "MATRIX_UTILS"; 

// Create a new matrix and allocate memory
matrix_t* create_matrix(int rows, int cols) {
    if (rows <= 0 || cols <= 0) {
        ESP_LOGE(TAG, "Invalid matrix dimensions: %dx%d", rows, cols);
        return NULL;
    }
    matrix_t *mat = (matrix_t*)malloc(sizeof(matrix_t));
    if (mat == NULL) {
        ESP_LOGE(TAG, "Failed to allocate matrix struct.");
        return NULL;
    }
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (float*)calloc(rows * cols, sizeof(float)); // Initialize with zeros
    if (mat->data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate matrix data.");
        free(mat);
        return NULL;
    }
    return mat;
}

// Free matrix memory
void free_matrix(matrix_t *mat) {
    if (mat) {
        if (mat->data) {
            free(mat->data);
        }
        free(mat);
    }
}

// Matrix multiplication (C = A * B)
matrix_t* multiply_matrices(const matrix_t *mat1, const matrix_t *mat2) {
    if (mat1->cols != mat2->rows) {
        ESP_LOGE(TAG, "Matrix dimensions mismatch for multiplication: %dx%d vs %dx%d",
                 mat1->rows, mat1->cols, mat2->rows, mat2->cols);
        return NULL;
    }

    matrix_t *result = create_matrix(mat1->rows, mat2->cols);
    if (result == NULL) return NULL;

    for (int i = 0; i < mat1->rows; i++) {
        for (int j = 0; j < mat2->cols; j++) {
            float sum = 0.0f;
            for (int k = 0; k < mat1->cols; k++) {
                sum += GET_ELEM(mat1, i, k) * GET_ELEM(mat2, k, j);
            }
            SET_ELEM(result, i, j, sum);
        }
    }
    return result;
}

// Matrix transpose (B = A^T)
matrix_t* transpose_matrix(const matrix_t *mat) {
    matrix_t *result = create_matrix(mat->cols, mat->rows);
    if (result == NULL) return NULL;

    for (int i = 0; i < mat->rows; i++) {
        for (int j = 0; j < mat->cols; j++) {
            SET_ELEM(result, j, i, GET_ELEM(mat, i, j));
        }
    }
    return result;
}

// Helper to copy a matrix (deep copy)
matrix_t* copy_matrix(const matrix_t *src) {
    matrix_t *dest = create_matrix(src->rows, src->cols);
    if (dest) {
        memcpy(dest->data, src->data, src->rows * src->cols * sizeof(float));
    }
    return dest;
}

// Scalar matrix multiplication
matrix_t* scalar_multiply_matrix(const matrix_t *mat, float scalar) {
    matrix_t *result = create_matrix(mat->rows, mat->cols);
    if (result == NULL) return NULL;

    for (int i = 0; i < mat->rows * mat->cols; i++) {
        result->data[i] = mat->data[i] * scalar;
    }
    return result;
}

// Vector subtraction (result = vec1 - vec2) - assumes column vectors
matrix_t* subtract_vectors(const matrix_t *vec1, const matrix_t *vec2) {
    if (vec1->rows != vec2->rows || vec1->cols != 1 || vec2->cols != 1) {
        ESP_LOGE(TAG, "Invalid dimensions for vector subtraction.");
        return NULL;
    }
    matrix_t *result = create_matrix(vec1->rows, 1);
    if (result == NULL) return NULL;
    for (int i = 0; i < vec1->rows; i++) {
        SET_ELEM(result, i, 0, GET_ELEM(vec1, i, 0) - GET_ELEM(vec2, i, 0));
    }
    return result;
}

// Vector addition (result = vec1 + vec2) - assumes column vectors
matrix_t* add_vectors(const matrix_t *vec1, const matrix_t *vec2) {
    if (vec1->rows != vec2->rows || vec1->cols != 1 || vec2->cols != 1) {
        ESP_LOGE(TAG, "Invalid dimensions for vector addition.");
        return NULL;
    }
    matrix_t *result = create_matrix(vec1->rows, 1);
    if (result == NULL) return NULL;
    for (int i = 0; i < vec1->rows; i++) {
        SET_ELEM(result, i, 0, GET_ELEM(vec1, i, 0) + GET_ELEM(vec2, i, 0));
    }
    return result;
}

// Dot product of two column vectors
float dot_product(const matrix_t *vec1, const matrix_t *vec2) {
    if (vec1->rows != vec2->rows || vec1->cols != 1 || vec2->cols != 1) {
        ESP_LOGE(TAG, "Invalid dimensions for dot product.");
        return NAN; // Not a Number
    }
    float sum = 0.0f;
    for (int i = 0; i < vec1->rows; i++) {
        sum += GET_ELEM(vec1, i, 0) * GET_ELEM(vec2, i, 0);
    }
    return sum;
}

// Print matrix for debugging
void print_matrix(const char* name, const matrix_t *mat) {
    if (mat == NULL) {
        ESP_LOGI(TAG, "%s: NULL", name);
        return;
    }
    ESP_LOGI(TAG, "%s (%dx%d):", name, mat->rows, mat->cols);
    for (int i = 0; i < mat->rows; i++) {
        char row_str[256] = {0}; // Buffer for a single row string
        int offset = snprintf(row_str, sizeof(row_str), "[");
        for (int j = 0; j < mat->cols; j++) {
            offset += snprintf(row_str + offset, sizeof(row_str) - offset, " %.4f", GET_ELEM(mat, i, j));
        }
        snprintf(row_str + offset, sizeof(row_str) - offset, " ]");
        ESP_LOGI(TAG, "%s", row_str);
    }
}

// Calculate the squared Frobenius norm (sum of squares of all elements) for a matrix/vector
float matrix_norm_squared(const matrix_t *mat) {
    if (mat == NULL || mat->data == NULL) return 0.0f;
    float sum_sq = 0.0f;
    for (int i = 0; i < mat->rows * mat->cols; i++) {
        sum_sq += mat->data[i] * mat->data[i];
    }
    return sum_sq;
}

// Inverts a square matrix using Gauss-Jordan elimination
// Returns NULL on failure (e.g., singular matrix)
matrix_t* inverse_matrix(const matrix_t *mat) {
    if (mat->rows != mat->cols) {
        ESP_LOGE(TAG, "Cannot invert a non-square matrix.");
        return NULL;
    }

    int n = mat->rows;
    matrix_t *identity = create_matrix(n, n);
    matrix_t *A_copy = copy_matrix(mat); // Work on a copy

    if (!identity || !A_copy) {
        free_matrix(identity);
        free_matrix(A_copy);
        return NULL;
    }

    // Initialize identity matrix
    for (int i = 0; i < n; i++) {
        SET_ELEM(identity, i, i, 1.0f);
    }

    // Augment A_copy with identity: [A | I]
    // We will perform operations on both simultaneously
    // No explicit augmentation needed, just apply ops to both.

    for (int i = 0; i < n; i++) {
        // Find pivot (largest absolute value in current column, below current row)
        int pivot_row = i;
        for (int k = i + 1; k < n; k++) {
            if (fabsf(GET_ELEM(A_copy, k, i)) > fabsf(GET_ELEM(A_copy, pivot_row, i))) {
                pivot_row = k;
            }
        }

        if (pivot_row != i) {
            // Swap rows in A_copy
            for (int j = 0; j < n; j++) {
                float temp = GET_ELEM(A_copy, i, j);
                SET_ELEM(A_copy, i, j, GET_ELEM(A_copy, pivot_row, j));
                SET_ELEM(A_copy, pivot_row, j, temp);
            }
            // Swap rows in identity
            for (int j = 0; j < n; j++) {
                float temp = GET_ELEM(identity, i, j);
                SET_ELEM(identity, i, j, GET_ELEM(identity, pivot_row, j));
                SET_ELEM(identity, pivot_row, j, temp);
            }
        }

        // Check for singular matrix
        float pivot = GET_ELEM(A_copy, i, i);
        if (fabsf(pivot) < 1e-9) { // Use a small epsilon for float comparison
            ESP_LOGE(TAG, "Matrix is singular or nearly singular (pivot = %f). Cannot invert.", pivot);
            free_matrix(A_copy);
            free_matrix(identity);
            return NULL;
        }

        // Normalize pivot row in A_copy and identity
        float inv_pivot = 1.0f / pivot;
        for (int j = 0; j < n; j++) {
            SET_ELEM(A_copy, i, j, GET_ELEM(A_copy, i, j) * inv_pivot);
            SET_ELEM(identity, i, j, GET_ELEM(identity, i, j) * inv_pivot);
        }

        // Eliminate other rows
        for (int k = 0; k < n; k++) {
            if (k != i) {
                float factor = GET_ELEM(A_copy, k, i);
                for (int j = 0; j < n; j++) {
                    SET_ELEM(A_copy, k, j, GET_ELEM(A_copy, k, j) - factor * GET_ELEM(A_copy, i, j));
                    SET_ELEM(identity, k, j, GET_ELEM(identity, k, j) - factor * GET_ELEM(identity, i, j));
                }
            }
        }
    }

    free_matrix(A_copy); // No longer needed
    return identity;     // This now holds the inverse
}

// Calculate determinant of a square matrix using LU decomposition
float matrix_determinant(const matrix_t *mat) {
    if (mat->rows != mat->cols) {
        ESP_LOGE(TAG, "Cannot calculate determinant of a non-square matrix.");
        return NAN;
    }

    int n = mat->rows;
    matrix_t *A_copy = copy_matrix(mat); // Work on a copy
    if (!A_copy) return NAN;

    float det = 1.0f;
    int sign = 1; // Track sign changes from row swaps

    for (int i = 0; i < n; i++) {
        // Find pivot
        int pivot_row = i;
        for (int k = i + 1; k < n; k++) {
            if (fabsf(GET_ELEM(A_copy, k, i)) > fabsf(GET_ELEM(A_copy, pivot_row, i))) {
                pivot_row = k;
            }
        }

        if (pivot_row != i) {
            // Swap rows
            for (int j = 0; j < n; j++) {
                float temp = GET_ELEM(A_copy, i, j);
                SET_ELEM(A_copy, i, j, GET_ELEM(A_copy, pivot_row, j));
                SET_ELEM(A_copy, pivot_row, j, temp);
            }
            sign *= -1; // Row swap changes sign
        }

        float pivot = GET_ELEM(A_copy, i, i);
        if (fabsf(pivot) < 1e-9) {
            // Matrix is singular
            free_matrix(A_copy);
            return 0.0f;
        }

        det *= pivot;

        // Eliminate below pivot
        for (int k = i + 1; k < n; k++) {
            float factor = GET_ELEM(A_copy, k, i) / pivot;
            for (int j = i; j < n; j++) {
                SET_ELEM(A_copy, k, j, GET_ELEM(A_copy, k, j) - factor * GET_ELEM(A_copy, i, j));
            }
        }
    }

    free_matrix(A_copy);
    return sign * det;
}


// Fit ellipsoid to magnetometer samples using least-squares
// Input: samples[N][3] = {x,y,z}
// Output: ellipsoid parameters (10 coefficients)
matrix_t* fit_ellipsoid(float (*samples)[3], int num_samples) {
    // Build design matrix D (N x 10)
    matrix_t *D = create_matrix(num_samples, 10);
    for (int i = 0; i < num_samples; i++) {
        float x = samples[i][0];
        float y = samples[i][1];
        float z = samples[i][2];
        SET_ELEM(D, i, 0, x*x);
        SET_ELEM(D, i, 1, y*y);
        SET_ELEM(D, i, 2, z*z);
        SET_ELEM(D, i, 3, 2*x*y);
        SET_ELEM(D, i, 4, 2*x*z);
        SET_ELEM(D, i, 5, 2*y*z);
        SET_ELEM(D, i, 6, 2*x);
        SET_ELEM(D, i, 7, 2*y);
        SET_ELEM(D, i, 8, 2*z);
        SET_ELEM(D, i, 9, 1.0f);
    }

    // Solve least-squares: D * params � 0
    // You can implement SVD here or export to Python/NumPy offline
    // Placeholder: return NULL until solver is implemented
    return D;
}
/*
// Extract hard-iron bias (center of ellipsoid)
void extract_bias_and_matrix(const float params[10],
                             float bias[3],
                             float soft_iron[3][3]) {
    // Build A matrix
    float A[3][3] = {
        {params[0], params[3], params[4]},
        {params[3], params[1], params[5]},
        {params[4], params[5], params[2]}
    };
    float b_vec[3] = {params[6], params[7], params[8]};

    // Solve center = -A^{-1} * b
    matrix_t *A_mat = create_matrix(3,3);
    matrix_t *b_mat = create_matrix(3,1);
    for (int i=0;i<3;i++) {
        for (int j=0;j<3;j++) SET_ELEM(A_mat,i,j,A[i][j]);
        SET_ELEM(b_mat,i,0,b_vec[i]);
    }
    matrix_t *A_inv = inverse_matrix(A_mat);
    matrix_t *center = multiply_matrices(A_inv, b_mat);

    bias[0] = -GET_ELEM(center,0,0);
    bias[1] = -GET_ELEM(center,1,0);
    bias[2] = -GET_ELEM(center,2,0);

    // TODO: compute soft-iron matrix from A (requires eigen decomposition)
    // For now, set identity
    memset(soft_iron, 0, sizeof(float)*9);
    soft_iron[0][0] = 1.0f;
    soft_iron[1][1] = 1.0f;
    soft_iron[2][2] = 1.0f;

    free_matrix(A_mat);
    free_matrix(b_mat);
    free_matrix(A_inv);
    free_matrix(center);
}
*/

// Extract hard-iron bias (center of ellipsoid)
void extract_bias_and_matrix(const float params[10],
                             float bias[3],
                             float soft_iron[3][3]) {
    // Build A matrix
    float A[3][3] = {
        {params[0], params[3], params[4]},
        {params[3], params[1], params[5]},
        {params[4], params[5], params[2]}
    };
    float b_vec[3] = {params[6], params[7], params[8]};

    // Create matrix_t wrappers
    matrix_t *A_mat = create_matrix(3,3);
    matrix_t *b_mat = create_matrix(3,1);
    for (int i=0;i<3;i++) {
        for (int j=0;j<3;j++) SET_ELEM(A_mat,i,j,A[i][j]);
        SET_ELEM(b_mat,i,0,b_vec[i]);
    }

    // Try to invert A
    matrix_t *A_inv = inverse_matrix(A_mat);
    if (!A_inv) {
        ESP_LOGE("CALIB_3D", "Calibration failed: singular matrix.");
        // Fallback: zero bias, identity soft-iron
        bias[0] = bias[1] = bias[2] = 0.0f;
        memset(soft_iron, 0, sizeof(float)*9);
        soft_iron[0][0] = soft_iron[1][1] = soft_iron[2][2] = 1.0f;

        free_matrix(A_mat);
        free_matrix(b_mat);
        return;
    }

    // Multiply A_inv * b
    matrix_t *center = multiply_matrices(A_inv, b_mat);
    if (!center) {
        ESP_LOGE("CALIB_3D", "Calibration failed: could not compute center.");
        bias[0] = bias[1] = bias[2] = 0.0f;
        memset(soft_iron, 0, sizeof(float)*9);
        soft_iron[0][0] = soft_iron[1][1] = soft_iron[2][2] = 1.0f;

        free_matrix(A_mat);
        free_matrix(b_mat);
        free_matrix(A_inv);
        return;
    }

    // Extract bias
    bias[0] = -GET_ELEM(center,0,0);
    bias[1] = -GET_ELEM(center,1,0);
    bias[2] = -GET_ELEM(center,2,0);

    // TODO: compute soft-iron matrix from A (requires eigen decomposition)
    // For now, set identity
    memset(soft_iron, 0, sizeof(float)*9);
    soft_iron[0][0] = 1.0f;
    soft_iron[1][1] = 1.0f;
    soft_iron[2][2] = 1.0f;

    // Cleanup
    free_matrix(A_mat);
    free_matrix(b_mat);
    free_matrix(A_inv);
    free_matrix(center);
}

