#include <gtest/gtest.h>
#include <linalg.h>

TEST(TestLinalg, MatMulVec)
{
    Matrix A;
    Vector b;
    Vector Ab;
    double *expected;

    int n = 12;
    expected = (double*)malloc(n * sizeof(double));
    Matrix_Init(&A, n, n);
    Vector_Init(&b, n);
    Vector_Init(&Ab, n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            MATRIX_AT(A, i, j) = (double)((i*n + j) % 5);
        }
        VECTOR_AT(b, i) = (double)(i % 5);
    }

    for (int i = 0; i < n; i++) {
        expected[i] = 0;
        for (int j = 0; j < n; j++) {
            expected[i] += MATRIX_AT(A, i, j) * VECTOR_AT(b, j);
        }
    }

    Matrix_MulVec(&A, &b, &Ab);
    
    for (int i = 0; i < n; i++) {
        EXPECT_DOUBLE_EQ(VECTOR_AT(Ab, i), expected[i]);
    }

    free(expected);
    Matrix_Free(&A);
    Vector_Free(&b);
    Vector_Free(&Ab);
}