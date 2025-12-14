#include "utils.h"
#include "solvers.h"

class TestPG : public TestSolver<true> {
public:
    void CheckExact(int n, double *A_arr, double *b_arr, ProjectFunc project, double *expected_x)
    {
        SetupMatrices(n, A_arr, b_arr);
        SolvePG(&A, &b, &x, project);
        ExpectExact(expected_x);
    }

    void CheckOK(int n, double *A_arr, double *b_arr, ProjectFunc project)
    {
        SetupMatrices(n, A_arr, b_arr);
        SolvePG(&A, &b, &x, project);
        ExpectOK();
    }
};

static void ProjectPositive(Vector *x)
{
    for (int i = 0; i < x->len; i++) {
        VECTOR_AT(*x, i) = std::max(0.0, VECTOR_AT(*x, i));
    }
}

static void ProjectNone(Matrix *x)
{
    return;
}

TEST_F(TestPG, Trivial)
{
    static double A_arr[] = {
        2, 1,
        1, 2
    };
    static double b_arr[] = {
        -5, -5
    };
    static double expected_x[] = {
        0, 0
    };

    CheckExact(2, A_arr, b_arr, ProjectPositive, expected_x);
}

TEST_F(TestPG, Identity)
{
    static double A_arr[] = {
        1, 0,
        0, 1
    };
    static double b_arr[] = {
        3, 4
    };
    static double expected_x[] = {
        3, 4
    };

    CheckExact(2, A_arr, b_arr, ProjectPositive, expected_x);
}

TEST_F(TestPG, SPD)
{
    static double A_arr[] = {
        4, -1,
        -1, 4
    };
    static double b_arr[] = {
        5, 6
    };

    CheckOK(2, A_arr, b_arr, ProjectPositive);
}

TEST_F(TestPG, ManySolutions)
{
    static double A_arr[] = {
        1, 1,
        1, 1
    };
    static double b_arr[] = {
        1, 1
    };

    CheckOK(2, A_arr, b_arr, ProjectPositive);
}

TEST_F(TestPG, Blocks)
{
    static double A_arr[] = {
        3, 1, 0, 0,
        1, 3, 0, 0,
        0, 0, 3, 1,
        0, 0, 1, 3
    };
    static double b_arr[] = {
        4, 5, 6, 7
    };
    
    CheckOK(4, A_arr, b_arr, ProjectPositive);
}
