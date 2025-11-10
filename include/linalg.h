#ifndef LINALG_H
#define LINALG_H

#include <math.h>

typedef struct {
    double x, y;
} Vec2;

static inline Vec2 Vec2_Add(Vec2 a, Vec2 b) { return (Vec2) { a.x + b.x, a.y + b.y }; }
static inline Vec2 Vec2_Sub(Vec2 a, Vec2 b) { return (Vec2) { a.x - b.x, a.y - b.y }; }
static inline Vec2 Vec2_Scale(Vec2 a, double s) { return (Vec2) { a.x * s, a.y * s }; }
static inline double Vec2_Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
static inline double Vec2_Length(Vec2 v) { return sqrtf(v.x * v.x + v.y * v.y); }
static inline Vec2 Vec2_Normalize(Vec2 v) { return Vec2_Scale(v, 1.0f / Vec2_Length(v)); }
static inline double Vec2_Cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

#define MATRIX_AT(mat, row, col) ((mat).data[(row) * (mat).cols + (col)])

typedef struct {
    int rows, cols;
    double *data;
} Matrix;

void Matrix_Free(Matrix *mat);
void Matrix_Init(Matrix *mat, int rows, int cols);
void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res);
void Matrix_InitTransposed(Matrix *src, Matrix *dst);
void Matrix_InitIdentity(Matrix *mat, int size);
void Matrix_Add(Matrix *A, Matrix *B, Matrix *res);
void Matrix_Put(Matrix *mat, Matrix *src, int row, int col);
void Matrix_Print(Matrix *mat);

#endif
