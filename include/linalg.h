#ifndef LINALG_H
#define LINALG_H

#include <math.h>

typedef struct {
    float a[4][4];
} Mat4f;

typedef struct {
    float x, y, z;
} Vec3f;

typedef struct {
    float x, y;
} Vec2f;

void Mat4_Ortho(Mat4f *mat, float left, float right, float bottom, float top);
void Mat4_Perspective(Mat4f *mat, float fov, float aspect, float near, float far);
void Mat4_LookAt(Mat4f *mat, Vec3f eye, Vec3f center, Vec3f up);
void Mat4_Identity(Mat4f *mat);
Vec3f Vec3_Cross(Vec3f a, Vec3f b);
Vec3f Vec3_Normalize(Vec3f v);

static inline Vec3f Vec3_Add(Vec3f a, Vec3f b) { return (Vec3f) { a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline Vec3f Vec3_Sub(Vec3f a, Vec3f b) { return (Vec3f) { a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline Vec3f Vec3_Scale(Vec3f a, float s) { return (Vec3f) { a.x * s, a.y * s, a.z * s }; }
static inline float Vec3_Dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float Vec3_Length(Vec3f v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

static inline Vec2f Vec2_Add(Vec2f a, Vec2f b) { return (Vec2f) { a.x + b.x, a.y + b.y }; }
static inline Vec2f Vec2_Sub(Vec2f a, Vec2f b) { return (Vec2f) { a.x - b.x, a.y - b.y }; }
static inline Vec2f Vec2_Scale(Vec2f a, float s) { return (Vec2f) { a.x * s, a.y * s }; }
static inline float Vec2_Dot(Vec2f a, Vec2f b) { return a.x * b.x + a.y * b.y; }
static inline float Vec2_Length(Vec2f v) { return sqrtf(v.x * v.x + v.y * v.y); }
static inline Vec2f Vec2_Normalize(Vec2f v) { return Vec2_Scale(v, 1.0f / Vec2_Length(v)); }

#define MATRIX_AT(mat, row, col) ((mat).data[(row) * (mat).cols + (col)])

typedef struct {
    int rows, cols;
    float *data;
} Matrix;

void Matrix_Free(Matrix *mat);
void Matrix_Init(Matrix *mat, int rows, int cols);
void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res);
void Matrix_InitTransposed(Matrix *src, Matrix *dst);
void Linang_Solve(Matrix *A, Matrix *b, Matrix *x);

#endif
