#ifndef BRICKS_SOLVERS_H
#define BRICKS_SOLVERS_H

#include "linalg.h"

typedef void (*ProjectFunc)(Matrix *x);

// Mz + q >= 0
// z >= 0
// z`(Mz + q) = 0
void SolveLemke(Matrix *M_in, Matrix *q_in, Matrix *z_out);

// Projected Gauss-Seidel
// Ax = b
void SolvePGS(Matrix *A, Matrix *b, Matrix *x, ProjectFunc project);

// Projected Gradient
// Ax = b
void SolvePG(Matrix *A, Matrix *b, Matrix *x, ProjectFunc project);

#endif
