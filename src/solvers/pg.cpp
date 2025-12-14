#include "solvers.h"
#include <cstdio>

#define MAX_ITER 128

void SolvePG(Matrix *A, Vector *b, Vector *x, ProjectFunc project)
{
    int n = A->cols;
    Vector tmp;
    Vector_Init(&tmp, n);
    
    double L = Matrix_InfinityNorm(A) * 1.1;
    
    for (int k = 0; k < MAX_ITER*n; k++) {
        // Ax - b;
        Matrix_MulVec(A, x, &tmp);
        Vector_MulAdd(&tmp, b, -1.0);
        
        // x -= (1/L) * gradient
        Vector_MulAdd(x, &tmp, -1.0/L);
        
        project(x);
    }
    
    Vector_Free(&tmp);
}