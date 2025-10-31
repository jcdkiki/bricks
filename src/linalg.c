#include "linalg.h"
#include <stdlib.h>

void Matrix_Init(Matrix *mat, int rows, int cols)
{
    mat->rows = rows;
    mat->cols = cols;
    mat->data = calloc(rows * cols, sizeof(double));
}

void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res)
{
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < B->cols; j++) {
            MATRIX_AT(*res, i, j) = 0.f;
            for (int k = 0; k < A->cols; k++) {
                MATRIX_AT(*res, i, j) += MATRIX_AT(*A, i, k) * MATRIX_AT(*B, k, j);
            }
        }
    }
}

void Matrix_Free(Matrix *mat)
{
    free(mat->data);
}

void Matrix_InitTransposed(Matrix *src, Matrix *dst)
{   
    Matrix_Init(dst, src->cols, src->rows);

    for (int i = 0; i < src->rows; i++) {
        for (int j = 0; j < src->cols; j++) {
            MATRIX_AT(*dst, j, i) = MATRIX_AT(*src, i, j);
        }
    }
}

void Matrix_InitIdentity(Matrix *mat, int size)
{
    Matrix_Init(mat, size, size);
    for (int i = 0; i < size; i++) {
        MATRIX_AT(*mat, i, i) = 1.f;
    }
}