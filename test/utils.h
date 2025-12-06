#ifndef BRICKS_TEST_UTILS_H
#define BRICKS_TEST_UTILS_H

#include <gtest/gtest.h>
#include "linalg.h"

template<bool Ax_eq_b>
class TestSolver : public ::testing::Test {
protected:
    Matrix A;
    Vector b, x;
public:
    void SetupMatrices(int n, double *A_arr, double *b_arr)
    {
        Matrix_Init(&A, n, n);
        Vector_Init(&b, n);
        Vector_Init(&x, n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                MATRIX_AT(A, i, j) = A_arr[i*n + j];
            }
            VECTOR_AT(b, i) = b_arr[i];
        }
    }

    void TearDown() override
    {
        Matrix_Free(&A);
        Vector_Free(&b);
        Vector_Free(&x);
    }

    void ExpectExact(double *expected_x)
    {
        for (int i = 0; i < x.len; i++) {
            EXPECT_EQ(VECTOR_AT(x, i), expected_x[i]);
        }
    }

    void ExpectOK()
    {
        int n = x.len;
        for (int i = 0; i < n; i++) {
            double a_i = 0;
            for (int j = 0; j < n; j++) {
                a_i += MATRIX_AT(A, i, j) * VECTOR_AT(x, j);
            }
            
            if constexpr (Ax_eq_b) {
                a_i -= VECTOR_AT(b, i);
            }
            else {
                a_i += VECTOR_AT(b, i);
            }

            double x_i = VECTOR_AT(x, i);
            double b_i = VECTOR_AT(b, i);

            EXPECT_FALSE(isnan(x_i));
            EXPECT_FALSE(isnan(a_i));
            EXPECT_GE(a_i, 0) << "a_i: " << a_i << ", x_i: " << x_i << ", b_i: " << b_i;
            EXPECT_GE(x_i, 0) << "a_i: " << a_i << ", x_i: " << x_i << ", b_i: " << b_i;
            EXPECT_EQ(a_i*x_i, 0) << "a_i: " << a_i << ", x_i: " << x_i << ", b_i: " << b_i;
        }
    }
};

#endif