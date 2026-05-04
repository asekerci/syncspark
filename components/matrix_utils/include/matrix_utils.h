/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * matrix_utils.h - Matrix mathematics utilities for SynchroSpark
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

#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <stdio.h> 
#include <stdbool.h> 
#include <esp_log.h> 
#include <stdlib.h> // For malloc/free
#include <math.h>   
#include <string.h> 

typedef struct {
    int rows;
    int cols;
    float *data; // Row-major order
} matrix_t;

// Get element at (row, col)
#define GET_ELEM(mat, row, col) (mat->data[(row) * (mat->cols) + (col)])
#define SET_ELEM(mat, row, col, val) (mat->data[(row) * (mat->cols) + (col)] = (val))

// Function prototypes
matrix_t *create_matrix(int rows, int cols);
matrix_t *copy_matrix(const matrix_t *src); // New helper prototype
matrix_t *multiply_matrices(const matrix_t *mat1, const matrix_t *mat2);
matrix_t *transpose_matrix(const matrix_t *mat);
matrix_t *inverse_matrix(const matrix_t *mat); // Using Gauss-Jordan
matrix_t *scalar_multiply_matrix(const matrix_t *mat, float scalar);
matrix_t *subtract_vectors(const matrix_t *vec1, const matrix_t *vec2);
matrix_t *add_vectors(const matrix_t *vec1, const matrix_t *vec2);
float dot_product(const matrix_t *vec1, const matrix_t *vec2);
float matrix_norm_squared(const matrix_t *mat); // For vector magnitude calculation
float matrix_determinant(const matrix_t *mat); // Calculate determinant for square matrices
void free_matrix(matrix_t *mat);
void print_matrix(const char *name, const matrix_t *mat); // Helper for debugging

// Ellipsoid fitting for magnetometer calibration
// Build design matrix and solve least-squares fit
matrix_t* fit_ellipsoid(float (*samples)[3], int num_samples);

// Extract hard-iron bias and soft-iron matrix from ellipsoid parameters
void extract_bias_and_matrix(const float params[10],
                             float bias[3],
                             float soft_iron[3][3]);


#endif // MATRIX_UTILS_H
