#ifndef BRICKS_SOLVERS_H
#define BRICKS_SOLVERS_H

#include "linalg.h"

typedef void (*ProjectFunc)(Vector *x);

// Mz + q >= 0
// z >= 0
// z`(Mz + q) = 0
void SolveLemke(Matrix *M_in, Vector *q_in, Vector *z_out);

// Projected Gauss-Seidel
// Ax = b
void SolvePGS(Matrix *A, Vector *b, Vector *x, ProjectFunc project);

// Projected Gradient
// Ax = b
void SolvePG(Matrix *A, Vector *b, Vector *x, ProjectFunc project);

#endif
