#include "linalg.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <glad/glad.h>

void Mat4_Ortho(Mat4f *mat, float left, float right, float bottom, float top)
{
    memset(mat, 0, sizeof(Mat4f));
    mat->a[0][0] = 2.f / (right - left);
    mat->a[1][1] = 2.f / (top - bottom);
    mat->a[2][2] = -1.f;
    mat->a[3][3] = 1.f;
    mat->a[3][0] = -(right + left) / (right - left);
    mat->a[3][1] = -(top + bottom) / (top - bottom);
}

void Mat4_Perspective(Mat4f *mat, float fov, float aspect, float near, float far)
{
    memset(mat, 0, sizeof(Mat4f));
    float f = 1.f / tanf(fov * M_PI / 360.f);
    mat->a[0][0] = f / aspect;
    mat->a[1][1] = f;
    mat->a[2][2] = (far + near) / (near - far);
    mat->a[2][3] = -1.f;
    mat->a[3][2] = (2.f * far * near) / (near - far);
}

void Mat4_LookAt(Mat4f *mat, Vec3f eye, Vec3f center, Vec3f up)
{
    Vec3f f = Vec3_Normalize(Vec3_Sub(center, eye));
    Vec3f s = Vec3_Normalize(Vec3_Cross(f, up));
    Vec3f u = Vec3_Cross(s, f);
    
    Mat4_Identity(mat);
    mat->a[0][0] = s.x;
    mat->a[1][0] = s.y;
    mat->a[2][0] = s.z;
    mat->a[0][1] = u.x;
    mat->a[1][1] = u.y;
    mat->a[2][1] = u.z;
    mat->a[0][2] = -f.x;
    mat->a[1][2] = -f.y;
    mat->a[2][2] = -f.z;
    mat->a[3][0] = -(s.x * eye.x + s.y * eye.y + s.z * eye.z);
    mat->a[3][1] = -(u.x * eye.x + u.y * eye.y + u.z * eye.z);
    mat->a[3][2] = f.x * eye.x + f.y * eye.y + f.z * eye.z;
}

void Mat4_Identity(Mat4f *mat)
{
    memset(mat, 0, sizeof(Mat4f));
    mat->a[0][0] = mat->a[1][1] = mat->a[2][2] = mat->a[3][3] = 1.f;
}

Vec3f Vec3_Cross(Vec3f a, Vec3f b)
{
    return (Vec3f){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3f Vec3_Normalize(Vec3f v)
{
    float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len > 0.f) {
        return (Vec3f){v.x / len, v.y / len, v.z / len};
    }
    return (Vec3f){0.f, 0.f, 0.f};
}

void Matrix_Init(Matrix *mat, int rows, int cols)
{
    mat->rows = rows;
    mat->cols = cols;
    mat->data = calloc(rows * cols, sizeof(float));
}

void Matrix_Mul(Matrix *A, Matrix *B, Matrix *res)
{
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < B->cols; j++) {
            MATRIX_AT(*res, i, j) = 0.f;
            for (int k = 0; k < A->cols; k++) {
                MATRIX_AT(*res, i, j) += MATRIX_AT(*A, i, k) * MATRIX_AT(*B, k, j);
            }
        }
    }
}

void Linang_Solve(Matrix *A, Matrix *b, Matrix *x)
{
    int n = A->rows;
    float *aug = malloc(n * (n + 1) * sizeof(float));
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * (n + 1) + j] = MATRIX_AT(*A, i, j);
        }
        aug[i * (n + 1) + n] = MATRIX_AT(*b, i, 0);
    }
    
    for (int i = 0; i < n; i++) {
        int pivot = i;
        for (int j = i + 1; j < n; j++) {
            if (fabsf(aug[j * (n + 1) + i]) > fabsf(aug[pivot * (n + 1) + i])) pivot = j;
        }
        if (pivot != i) {
            for (int j = 0; j <= n; j++) {
                float tmp = aug[i * (n + 1) + j];
                aug[i * (n + 1) + j] = aug[pivot * (n + 1) + j];
                aug[pivot * (n + 1) + j] = tmp;
            }
        }
        
        for (int j = i + 1; j < n; j++) {
            float factor = aug[j * (n + 1) + i] / aug[i * (n + 1) + i];
            for (int k = i; k <= n; k++) {
                aug[j * (n + 1) + k] -= factor * aug[i * (n + 1) + k];
            }
        }
    }
    
    for (int i = n - 1; i >= 0; i--) {
        MATRIX_AT(*x, i, 0) = aug[i * (n + 1) + n];
        for (int j = i + 1; j < n; j++) {
            MATRIX_AT(*x, i, 0) -= aug[i * (n + 1) + j] * MATRIX_AT(*x, j, 0);
        }
        MATRIX_AT(*x, i, 0) /= aug[i * (n + 1) + i];
    }
    
    free(aug);
}

void Matrix_Free(Matrix *mat)
{
    free(mat->data);
}

void Matrix_InitTransposed(Matrix *src, Matrix *dst)
{   
    Matrix_Init(dst, src->cols, src->rows);

    for (int i = 0; i < src->rows; i++) {
        for (int j = 0; j < src->cols; j++) {
            MATRIX_AT(*dst, j, i) = MATRIX_AT(*src, i, j);
        }
    }
}