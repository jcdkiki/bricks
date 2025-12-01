#include "solvers.h"
#include <cstdio>

#define MAX_ITER 128

void SolvePG(Matrix *A, Matrix *b, Matrix *x, ProjectFunc project)
{
    int n = A->cols;
    Matrix grad, Ax;
    Matrix_Init(&grad, n, 1);
    Matrix_Init(&Ax, n, 1);
    
    double L = Matrix_InfinityNorm(A) * 1.1;
    
    for (int k = 0; k < MAX_ITER*n; k++) {
        // gradient Ax - b;
        Matrix_Mul(A, x, &Ax);
        for (int i = 0; i < n; i++) {
            MATRIX_AT(grad, i, 0) = MATRIX_AT(Ax, i, 0) - MATRIX_AT(*b, i, 0);
        }
        
        // x -= (1/L) * gradient
        for (int i = 0; i < n; i++) {
            MATRIX_AT(*x, i, 0) -= (1.0/L) * MATRIX_AT(grad, i, 0);
        }
        
        project(x);
    }
    
    Matrix_Free(&grad);
    Matrix_Free(&Ax);
}