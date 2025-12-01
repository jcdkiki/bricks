#ifndef LINALG_H
#define LINALG_H

#include <math.h>

struct Vec2 {
    double x, y;
};

struct Vec3 {
    double x, y, z;
};

struct Vec4 {
    double x, y, z, w;
};

struct Matrix3 {
    float a[9];
};

struct Matrix4 {
    float a[16];
};

static inline Vec2 Vec2_Add(Vec2 a, Vec2 b) { return (Vec2) { a.x + b.x, a.y + b.y }; }
static inline Vec2 Vec2_Sub(Vec2 a, Vec2 b) { return (Vec2) { a.x - b.x, a.y - b.y }; }
static inline Vec2 Vec2_Scale(Vec2 a, double s) { return (Vec2) { a.x * s, a.y * s }; }
static inline double Vec2_Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
static inline double Vec2_Length(Vec2 v) { return sqrtf(v.x * v.x + v.y * v.y); }
static inline Vec2 Vec2_Normalize(Vec2 v) { return Vec2_Scale(v, 1.0f / Vec2_Length(v)); }
static inline double Vec2_Cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

static inline double Vec3_Dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3 Vec3_Add(Vec3 a, Vec3 b) { return (Vec3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vec3 Vec3_Sub(Vec3 a, Vec3 b) { return (Vec3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline Vec3 Vec3_Scale(Vec3 a, double s) { return (Vec3){a.x*s, a.y*s, a.z*s}; }
static inline double Vec3_Length(Vec3 a) { return sqrt(Vec3_Dot(a, a)); }

static inline Vec3 Vec3_Normalize(Vec3 a)
{
    double len = Vec3_Length(a);
    if (len == 0) return (Vec3){0,0,0};
    return Vec3_Scale(a, 1.0/len);
}

static inline Vec3 Vec3_Cross(Vec3 a, Vec3 b)
{
    return (Vec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

#define MATRIX_AT(mat, row, col) ((mat).data[(row) * (mat).cols + (col)])

struct Matrix {
    int rows, cols;
    double *data;
};

void Matrix_Free(Matrix *mat);
void Matrix_Init(Matrix *mat, int rows, int cols);
void Matrix_MulInit(Matrix *A, Matrix *B, Matrix *res);
void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res);
void Matrix_InitTransposed(Matrix *src, Matrix *dst);
void Matrix_InitIdentity(Matrix *mat, int size);
void Matrix_Add(Matrix *A, Matrix *B, Matrix *res);
void Matrix_Put(Matrix *mat, Matrix *src, int row, int col);
void Matrix_Print(Matrix *mat);
void Matrix_PrintTransposed(Matrix *mat);
double Matrix_Dot(Matrix *A, Matrix *B);

static inline void Matrix_Negate(Matrix *A)
{
    for (int i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = -A->data[i];
    }
}

static inline void Matrix_FillZeros(Matrix *A)
{
    for (int i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = 0.0;
    }
}

static inline double Matrix_Norm(Matrix *A)
{
    double res = 0.0;
    for (int i = 0; i < A->rows * A->cols; i++) {
        res += A->data[i] * A->data[i];
    }
    return res;
}

static inline double Matrix_InfinityNorm(Matrix *A)
{
    double max_row_sum = 0.0;
    for (int i = 0; i < A->rows; i++) {
        double row_sum = 0.0;
        for (int j = 0; j < A->cols; j++) {
            row_sum += fabs(MATRIX_AT(*A, i, j));
        }
        if (row_sum > max_row_sum) {
            max_row_sum = row_sum;
        }
    }
    return max_row_sum;
}

// (Matrix*, const char*)*count
void Matrix_PrintMany(int count, ...);

#endif
