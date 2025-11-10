#include "linalg.h"
#include <stdio.h>
#include <stdlib.h>

void Matrix_Init(Matrix *mat, int rows, int cols)
{
    mat->rows = rows;
    mat->cols = cols;
    mat->data = calloc(rows * cols, sizeof(double));
}

void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res)
{
    Matrix_Init(res, A->rows, B->cols);

    if (A->cols != B->rows) {
        printf("size mismatch. matrices %dx%d * %dx%d\n", A->rows, A->cols, B->rows, B->cols);
        return;
    }

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

void Matrix_Add(Matrix *A, Matrix *B, Matrix *res)
{
    if (A->rows != B->rows || A->cols != B->cols) {
        printf("size mismatch. matrices %dx%d + %dx%d\n", A->rows, A->cols, B->rows, B->cols);
        return;
    }

    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            MATRIX_AT(*res, i, j) = MATRIX_AT(*A, i, j) + MATRIX_AT(*B, i, j);
        }
    }
}

void Matrix_Put(Matrix *mat, Matrix *src, int row, int col)
{
    for (int i = 0; i < src->rows; i++) {
        for (int j = 0; j < src->cols; j++) {
            MATRIX_AT(*mat, row + i, col + j) = MATRIX_AT(*src, i, j);
        }
    }
}

void Matrix_Print(Matrix *mat)
{
    for (int i = 0; i < mat->rows; i++) {
        for (int j = 0; j < mat->cols; j++) {
            printf("%f ", MATRIX_AT(*mat, i, j));
        }
        printf("\n");
    }
}
