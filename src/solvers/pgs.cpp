#include "solvers.h"
#include <cstdio>

#define MAX_ITER 100

double CalcError(Matrix *A, Matrix *b, Matrix *x)
{
    int n = A->cols;
    double err_sum = 0.0;
    for (int i = 0; i < n; i++) {
        double a = -MATRIX_AT(*b, i, 0);
        for (int j = 0; j < n; j++) {
            a += MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
        }
        err_sum += a;
    }
    return err_sum;
}

void SolvePGS(Matrix *A, Matrix *b, Matrix *x, ProjectFunc project)
{
    int n = A->cols;
    for (int i = 0; i < n; i++) MATRIX_AT(*x, i, 0) = 0;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        for (int i = 0; i < n; i++) {
            double sum = 0.0;
            
            for (int j = 0; j < i; j++) {
                sum += MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
            }
            for (int j = i + 1; j < n; j++) {
                sum += MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
            }
            
            MATRIX_AT(*x, i, 0) = (MATRIX_AT(*b, i, 0) - sum) / MATRIX_AT(*A, i, i);
        }

        project(x);
    }
}