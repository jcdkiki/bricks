#include "linalg.h"
#include <cmath>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>

void Matrix_Init(Matrix *mat, int rows, int cols)
{
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (double*)calloc(rows * cols, sizeof(double));
}

void Matrix_MulVec(Matrix *A, Vector *b, Vector *c)
{
    int n = A->rows;
    for (int i = 0; i < n; i++) {
        double s = 0.0;
        for (int j = 0; j < n; j++) {
            s += MATRIX_AT(*A, i, j) * VECTOR_AT(*b, j);
        }
        VECTOR_AT(*c, i) = s;
    }
}

void Matrix_Free(Matrix *mat)
{
    free(mat->data);
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
            printf("%4.01lf ", MATRIX_AT(*mat, i, j));
        }
        printf("\n");
    }
}

void Vector_Print(Vector *vec)
{
    for (int i = 0; i < vec->len; i++) {
        printf("%4.01lf ", VECTOR_AT(*vec, i));
    }
    printf("\n");
}

double Vector_Dot(Vector *A, Vector *B)
{
    if (A->len != B->len) {
        printf("A and b have different sizes\n");
        return 0.0;
    }

    int n = A->len;
    double res = 0.0;
    for (int i = 0; i < n; i++) {
        res += VECTOR_AT(*A, i) * VECTOR_AT(*B, i); 
    }
    return res;
}

void Vector_Free(Vector *vec)
{
    free(vec->data);
}

void Vector_Init(Vector *vec, int len)
{
    vec->len = len;
    vec->data = (double*)calloc(len, sizeof(double));
}

void Matrix_Negate(Matrix *A)
{
    for (int i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = -A->data[i];
    }
}

void Matrix_FillZeros(Matrix *A)
{
    memset(A->data, 0, A->rows * A->cols * sizeof(double));
}

double Matrix_InfinityNorm(Matrix *A)
{
    double max_row_sum = 0.0;
    for (int i = 0; i < A->rows; i++) {
        double row_sum = 0.0;
        for (int j = 0; j < A->cols; j++) {
            row_sum += fabs(MATRIX_AT(*A, i, j));
        }
        if (row_sum > max_row_sum) {
            max_row_sum = row_sum;
        }
    }
    return max_row_sum;
}

void Vector_MulAdd(Vector *dst, Vector *src, double scale)
{
    for (int i = 0; i < dst->len; i++) {
        VECTOR_AT(*dst, i) += VECTOR_AT(*src, i) * scale;
    }
}