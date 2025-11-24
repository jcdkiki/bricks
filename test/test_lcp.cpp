#include <gtest/gtest.h>
#include "lemke.h"

TEST(LCPTest, Trivial)
{
    Matrix A, b, x;
    A.rows = 2; A.cols = 2;
    b.rows = 2; b.cols = 1;
    x.rows = 2; x.cols = 1;
    
    static double A_arr[] = {
        2, 1,
        1, 2
    };
    static double b_arr[] = {
        5, 5
    };
    static double x_arr[] = {
        0, 0
    };

    A.data = A_arr;
    b.data = b_arr;
    x.data = x_arr;

    SolveLemke(&A, &b, &x);
    EXPECT_EQ(x.data[0], 0);
    EXPECT_EQ(x.data[1], 0);
}

TEST(LCPTest, Identity)
{
    Matrix A, b, x;
    A.rows = 2; A.cols = 2;
    b.rows = 2; b.cols = 1;
    x.rows = 2; x.cols = 1;
    
    static double A_arr[] = {
        1, 0,
        0, 1
    };
    static double b_arr[] = {
        -3, -4
    };
    static double x_arr[] = {
        0, 0
    };

    A.data = A_arr;
    b.data = b_arr;
    x.data = x_arr;

    SolveLemke(&A, &b, &x);
    EXPECT_EQ(x.data[0], 3);
    EXPECT_EQ(x.data[1], 4);
}

TEST(LCPTest, SPD)
{
    Matrix A, b, x;
    A.rows = 2; A.cols = 2;
    b.rows = 2; b.cols = 1;
    x.rows = 2; x.cols = 1;
    
    static double A_arr[] = {
        4, -1,
        -1, 4
    };
    static double b_arr[] = {
        -5, -6
    };
    static double x_arr[] = {
        0, 0
    };

    A.data = A_arr;
    b.data = b_arr;
    x.data = x_arr;

    SolveLemke(&A, &b, &x);
    int a0 = 4*x.data[0] - x.data[1] + b_arr[0];
    int a1 = -x.data[0] + 4*x.data[1] + b_arr[1];

    EXPECT_GE(a0, 0);
    EXPECT_GE(a1, 0);
    EXPECT_GE(x.data[0], 0);
    EXPECT_GE(x.data[1], 0);
    EXPECT_EQ(a0*x.data[0], 0);
    EXPECT_EQ(a1*x.data[1], 0);
}

TEST(LCPTest, ManySolutions)
{
    Matrix A, b, x;
    A.rows = 2; A.cols = 2;
    b.rows = 2; b.cols = 1;
    x.rows = 2; x.cols = 1;
    
    static double A_arr[] = {
        1, 1,
        1, 1
    };
    static double b_arr[] = {
        -1, -1
    };
    static double x_arr[] = {
        0, 0
    };

    A.data = A_arr;
    b.data = b_arr;
    x.data = x_arr;

    SolveLemke(&A, &b, &x);
    int a0 = x.data[0] + x.data[1] + b_arr[0];
    int a1 = x.data[0] + x.data[1] + b_arr[1];

    EXPECT_GE(a0, 0);
    EXPECT_GE(a1, 0);
    EXPECT_GE(x.data[0], 0);
    EXPECT_GE(x.data[1], 0);
    EXPECT_EQ(a0*x.data[0], 0);
    EXPECT_EQ(a1*x.data[1], 0);
}

TEST(LCPTest, Blocks)
{
    Matrix A, b, x;
    A.rows = 4; A.cols = 4;
    b.rows = 4; b.cols = 1;
    x.rows = 4; x.cols = 1;
    
    static double A_arr[] = {
        3, 1, 0, 0,
        1, 3, 0, 0,
        0, 0, 3, 1,
        0, 0, 1, 3
    };
    static double b_arr[] = {
        -4, -5, -6, -7
    };
    static double x_arr[] = {
        0, 0, 0, 0
    };

    A.data = A_arr;
    b.data = b_arr;
    x.data = x_arr;

    SolveLemke(&A, &b, &x);
    int a0 = 3*x.data[0] + x.data[1] + b_arr[0];
    int a1 = x.data[0] + 3*x.data[1] + b_arr[1];
    int a2 = 3*x.data[2] + x.data[3] + b_arr[2];
    int a3 = x.data[2] + 3*x.data[3] + b_arr[3];

    EXPECT_GE(a0, 0);
    EXPECT_GE(a1, 0);
    EXPECT_GE(a2, 0);
    EXPECT_GE(a3, 0);
    EXPECT_GE(x.data[0], 0);
    EXPECT_GE(x.data[1], 0);
    EXPECT_GE(x.data[2], 0);
    EXPECT_GE(x.data[3], 0);
    EXPECT_EQ(a0*x.data[0], 0);
    EXPECT_EQ(a1*x.data[1], 0);
    EXPECT_EQ(a2*x.data[2], 0);
    EXPECT_EQ(a3*x.data[3], 0);
}
