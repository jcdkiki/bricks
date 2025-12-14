#include "utils.h"
#include "solvers.h"

class TestLemke : public TestSolver<false> {
public:
    void CheckExact(int n, double *A_arr, double *b_arr, double *expected_x)
    {
        SetupMatrices(n, A_arr, b_arr);
        SolveLemke(&A, &b, &x);
        ExpectExact(expected_x);
    }

    void CheckOK(int n, double *A_arr, double *b_arr)
    {
        SetupMatrices(n, A_arr, b_arr);
        SolveLemke(&A, &b, &x);
        ExpectOK();
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
