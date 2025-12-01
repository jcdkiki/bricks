#include "linalg.h"
#include <cstdarg>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>

void Matrix_Init(Matrix *mat, int rows, int cols)
{
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (double*)calloc(rows * cols, sizeof(double));
}

void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res)
{
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

void Matrix_MulInit(Matrix *A, Matrix *B, Matrix *res)
{
    Matrix_Init(res, A->rows, B->cols);
    Matrix_Mul(A, B, res);
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
            printf("%4.01lf ", MATRIX_AT(*mat, i, j));
        }
        printf("\n");
    }
}

void Matrix_PrintTransposed(Matrix *mat)
{
    for (int i = 0; i < mat->cols; i++) {
        for (int j = 0; j < mat->rows; j++) {
            printf("%4.01lf ", MATRIX_AT(*mat, j, i));
        }
        printf("\n");
    }
}

double Matrix_Dot(Matrix *A, Matrix *B)
{
    if (A->cols != 1 || B->cols != 1) {
        printf("A and B should be vectors\n");
        return 0.0;
    }

    if (A->rows != B->rows) {
        printf("A and b have different sizes\n");
        return 0.0;
    }

    int n = A->rows;
    double res = 0.0;
    for (int i = 0; i < n; i++) {
        res += MATRIX_AT(*A, i, 0) * MATRIX_AT(*B, i, 0); 
    }
    return res;
}

#define N_DIGITS 1
#define NUM_SIZE (3 + N_DIGITS)
#define COL_SIZE (NUM_SIZE + 1)
#define NUM_FMT "%4.01lf"

// (Matrix*, const char*)*count
void Matrix_PrintMany(int count, ...)
{
    va_list args;
    
    int max_rows = 0;

    va_start(args, count);
    for (int i = 0; i < count; ++i)
    {
        Matrix *A = va_arg(args, Matrix*);
        const char *name = va_arg(args, const char*);
        printf("%s", name);

        int width = A->cols * COL_SIZE + 3;
        int cur_width = strlen(name);
        while (cur_width != width) {
            fputc(' ', stdout);
            cur_width++;
        }

        if (max_rows < A->rows) max_rows = A->rows;
    }
    va_end(args);
    printf("\n");
    
    for (int row = 0; row < max_rows; row++) {
        va_start(args, count);
        for (int i = 0; i < count; i++)
        {
            Matrix *A = va_arg(args, Matrix*);
            const char *name = va_arg(args, const char*);
            (void)name;

            if (row >= A->rows) {
                int width = A->cols * COL_SIZE + 3;
                while (width--) fputc(' ', stdout);
            }
            else {
                for (int j = 0; j < A->cols; j++) {
                    printf(NUM_FMT " ", MATRIX_AT(*A, row, j));
                }
                printf("   ");
            }
        }
        va_end(args);
        printf("\n");
    }
}