#include <gtest/gtest.h>
#include "solvers.h"

class TestLemke : public ::testing::Test {
    Matrix A, b, x;
public:
    void SetupMatrices(int n, double *A_arr, double *b_arr)
    {
        A.rows = n; A.cols = n;
        b.rows = n; b.cols = 1;
        x.rows = n; x.cols = 1;
        A.data = A_arr;
        b.data = b_arr;
        x.data = (double*)calloc(n, sizeof(double));
    }

    void CheckExact(int n, double *A_arr, double *b_arr, double *expected_x)
    {
        SetupMatrices(n, A_arr, b_arr);
        SolveLemke(&A, &b, &x);

        for (int i = 0; i < n; i++) {
            EXPECT_EQ(x.data[i], expected_x[i]);
        }
        free(x.data);
    }

    void CheckOK(int n, double *A_arr, double *b_arr)
    {
        SetupMatrices(n, A_arr, b_arr);
        SolveLemke(&A, &b, &x);

        for (int i = 0; i < n; i++) {
            double a_i = 0;
            for (int j = 0; j < n; j++) {
                a_i += MATRIX_AT(A, i, j) * MATRIX_AT(x, j, 0);
            }
            a_i += MATRIX_AT(b, i, 0);

            double x_i = MATRIX_AT(x, i, 0);
            
            EXPECT_GE(a_i, 0);
            EXPECT_GE(x_i, 0);
            EXPECT_EQ(a_i*x_i, 0) << "a_i: " << a_i << ", x_i: " << x_i;
        }
        free(x.data);
    }
};

TEST_F(TestLemke, Trivial)
{
    static double A_arr[] = {
        2, 1,
        1, 2
    };
    static double b_arr[] = {
        5, 5
    };
    static double expected_x[] = {
        0, 0
    };

    CheckExact(2, A_arr, b_arr, expected_x);
}

TEST_F(TestLemke, Identity)
{
    static double A_arr[] = {
        1, 0,
        0, 1
    };
    static double b_arr[] = {
        -3, -4
    };
    static double expected_x[] = {
        3, 4
    };

    CheckExact(2, A_arr, b_arr, expected_x);
}

TEST_F(TestLemke, SPD)
{
    static double A_arr[] = {
        4, -1,
        -1, 4
    };
    static double b_arr[] = {
        -5, -6
    };

    CheckOK(2, A_arr, b_arr);
}

TEST_F(TestLemke, ManySolutions)
{
    static double A_arr[] = {
        1, 1,
        1, 1
    };
    static double b_arr[] = {
        -1, -1
    };

    CheckOK(2, A_arr, b_arr);
}

TEST_F(TestLemke, Blocks)
{
    static double A_arr[] = {
        3, 1, 0, 0,
        1, 3, 0, 0,
        0, 0, 3, 1,
        0, 0, 1, 3
    };
    static double b_arr[] = {
        -4, -5, -6, -7
    };
    
    CheckOK(4, A_arr, b_arr);
}
