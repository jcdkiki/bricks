#include "lemke.h"
#include <cfloat>
#include <cstdio>

void SolveLemke(Matrix *M_in, Matrix *q_in, Matrix *z_out)
{
    int n = M_in->rows;
    int cols = 2 * n + 2;
    
    Matrix tab;
    Matrix_Init(&tab, n, cols);
    
    int *basis = (int*)malloc(n * sizeof(int));
    for (int i = 0; i < n; i++) basis[i] = i;

    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            MATRIX_AT(tab, r, c) = (r == c) ? 1.0 : 0.0;
        }
        for (int c = 0; c < n; c++) {
            MATRIX_AT(tab, r, n + c) = -MATRIX_AT(*M_in, r, c);
        }
        
        MATRIX_AT(tab, r, 2 * n) = -1.0;
        MATRIX_AT(tab, r, 2 * n + 1) = MATRIX_AT(*q_in, r, 0);
    }

    int min_row = -1;
    double min_val = 0.0;
    for (int i = 0; i < n; i++) {
        double val = MATRIX_AT(tab, i, 2 * n + 1);
        if (val < min_val) {
            min_val = val;
            min_row = i;
        }
    }

    if (min_row == -1) {
        for (int i = 0; i < n; i++) MATRIX_AT(*z_out, i, 0) = 0.0;
        Matrix_Free(&tab);
        free(basis);
        return;
    }

    int entering_var = 2 * n;
    int max_iter = 50;

    for (int iter = 0; iter < max_iter; iter++) {
        int pivot_row = min_row;
        double pivot_val = MATRIX_AT(tab, pivot_row, entering_var);
        
        for (int c = 0; c < cols; c++) {
            MATRIX_AT(tab, pivot_row, c) /= pivot_val;
        }

        for (int r = 0; r < n; r++) {
            if (r != pivot_row) {
                double factor = MATRIX_AT(tab, r, entering_var);
                if (factor != 0.0) {
                    for (int c = 0; c < cols; c++) {
                        MATRIX_AT(tab, r, c) -= factor * MATRIX_AT(tab, pivot_row, c);
                    }
                }
            }
        }

        int leaving_var = basis[pivot_row];
        basis[pivot_row] = entering_var;

        if (leaving_var == 2 * n) {
            break;
        }

        if (leaving_var < n) {
            entering_var = leaving_var + n;
        } else {
            entering_var = leaving_var - n;
        }

        min_row = -1;
        min_val = DBL_MAX;

        for (int r = 0; r < n; r++) {
            double coeff = MATRIX_AT(tab, r, entering_var);
            if (coeff > 1e-8) { 
                double ratio = MATRIX_AT(tab, r, 2 * n + 1) / coeff;
                if (ratio < min_val) {
                    min_val = ratio;
                    min_row = r;
                }
            }
        }

        if (min_row == -1) {
            printf("unbounded\n");
            break;
        }
    }

    for (int i = 0; i < n; i++) MATRIX_AT(*z_out, i, 0) = 0.0;

    for (int r = 0; r < n; r++) {
        int var = basis[r];
        if (var >= n && var < 2 * n) {
            int z_idx = var - n;
            MATRIX_AT(*z_out, z_idx, 0) = MATRIX_AT(tab, r, 2 * n + 1);
        }
    }

    Matrix_Free(&tab);
    free(basis);
}
