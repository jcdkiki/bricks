#include "physics.h"

#include <cstring>
#include <float.h>
#include "linalg.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lemke.h"

#define GRAVITY 9.8

std::vector<Body> bodies;
std::vector<Contact> contacts;

void Phys_SaveToFile(const char *filename)
{
    FILE *f = fopen(filename, "wb");
    if (f == nullptr) {
        printf("Failed to open file %s\n", filename);
        return;
    }

    int n_bodies = bodies.size();
    int n_contacts = contacts.size();
    fwrite(&n_bodies, sizeof(int), 1, f);
    fwrite(&n_contacts, sizeof(int), 1, f);
    fwrite(bodies.data(), sizeof(Body), n_bodies, f);

    for (int i = 0; i < n_contacts; i++) {
        fwrite(&contacts[i].i, sizeof(int), 1, f);
        fwrite(&contacts[i].j, sizeof(int), 1, f);
        fwrite(&contacts[i].mu, sizeof(double), 1, f);
        fwrite(&contacts[i].pos, sizeof(Vec3), 1, f);
        fwrite(&contacts[i].angles, sizeof(Vec3), 1, f);  
    }
    
    fclose(f);
}

void Phys_ReadFromFile(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (f == nullptr) {
        printf("Failed to open file %s\n", filename);
        return;
    }

    int n_bodies;
    int n_contacts;
    fread(&n_bodies, sizeof(int), 1, f);
    fread(&n_contacts, sizeof(int), 1, f);
    
    bodies.resize(n_bodies);
    contacts.resize(n_contacts);
    fread(bodies.data(), sizeof(Body), n_bodies, f);
    
    memset(contacts.data(), 0, n_contacts * sizeof(Contact));
    for (int i = 0; i < n_contacts; i++) {
        fread(&contacts[i].i, sizeof(int), 1, f);
        fread(&contacts[i].j, sizeof(int), 1, f);
        fread(&contacts[i].mu, sizeof(double), 1, f);
        fread(&contacts[i].pos, sizeof(Vec3), 1, f);
        fread(&contacts[i].angles, sizeof(Vec3), 1, f);  
    }
}

void ProjectForces(Matrix *A, int axes, int forces)
{
    Matrix_Init(A, contacts.size(), contacts.size());
    for (int k = 0; k < contacts.size(); k++) {
        for (int l = 0; l < contacts.size(); l++) {
            Contact *ck = &contacts[k], *cl = &contacts[l];
            Body *bki = &bodies[ck->i], *bkj = &bodies[ck->j];
            double mi = bki->mass, mj = bkj->mass;
            
            Vec3 force_axis = cl->axes[forces];
            Vec3 project_to = ck->axes[axes];
            double nk_nl = Vec3_Dot(force_axis, project_to);

            if (l == k) {
                MATRIX_AT(*A, k, l) = nk_nl / mi + nk_nl / mj;
            }
            else if (cl->j == ck->j) MATRIX_AT(*A, k, l) = nk_nl / mj;
            else if (cl->i == ck->j) MATRIX_AT(*A, k, l) = -nk_nl / mj;
            else if (cl->i == ck->i) MATRIX_AT(*A, k, l) = nk_nl / mi;
            else if (cl->j == ck->i) MATRIX_AT(*A, k, l) = -nk_nl / mi;
        }
    }
}

/*
static void Verify(Matrix *A, Matrix *b, Matrix *x)
{
    int M = contacts.size();
    for (int i = 0; i < M; i++) {
        double an = MATRIX_AT(*b, i, 0);
        double fn = MATRIX_AT(*x, i, 0);
        for (int j = 0; j < M; j++) {
            an += MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
        }

        if (fn < -1e-6)
            printf("error in contact %d: f_n = %lf < 0\n", i, fn);
        if (an < -1e-6)
            printf("error in contact %d: a_n = %lf < 0\n", i, an);
        if (an * fn < -1e-6)
            printf("error in contact %d: f_n*a_n = %lf*%lf = %lf != 0\n", i, fn, an, fn*an);

        int cur_row = M;
        for (int j = 0; j < N_TANGENTS; j++) {
            int plus_row = cur_row + i;
            int minus_row = cur_row + M + i;
            double ft_plus = MATRIX_AT(*x, plus_row, 0);
            double ft_minus = MATRIX_AT(*x, minus_row, 0);

            double at_plus = MATRIX_AT(*b, plus_row, 0);
            double at_minus = MATRIX_AT(*b, minus_row, 0);

            for (int k = 0; k < M; k++) {
                at_plus += MATRIX_AT(*A, plus_row, k) * MATRIX_AT(*x, k, 0);
                at_minus += MATRIX_AT(*A, minus_row, k) * MATRIX_AT(*x, k, 0);
            }

            if (ft_plus < -1e-6) printf("error in contact %d tangent %d+: f_t = %lf < 0\n", i, j, ft_plus);
            if (ft_minus < -1e-6) printf("error in contact %d tangent %d-: f_t = %lf < 0\n", i, j, ft_minus);
            if (at_plus < -1e-6) printf("error in contact %d tangent %d+: a_t = %lf < 0\n", i, j, at_plus);
            if (at_minus < -1e-6) printf("error in contact %d tangent %d-: a_t = %lf < 0\n", i, j, at_minus);
            if (at_plus * ft_plus > 1e-6)
                printf("error in contact %d tangent %d+: f_t*a_t = %lf*%lf = %lf != 0\n", i, j, ft_plus, at_plus, ft_plus*at_plus);
            if (at_minus * ft_minus > 1e-6)
                printf("error in contact %d tangent %d-: f_t*a_t = %lf*%lf = %lf != 0\n", i, j, ft_minus, at_minus, ft_minus*at_minus);
            
            cur_row += 2*M;
        }
    }
}*/

static void Solve(Matrix *x)
{  
    int M = contacts.size();
    int MM = (2 + N_TANGENTS*2)*M;
    int cur_row = 0;

    Matrix I;
    Matrix_Init(&I, M, M);
    for (int i = 0; i < M; i++) MATRIX_AT(I, i, i) = 1.0;

    Matrix A, b;
    Matrix_Init(&A, MM, MM);
    Matrix_Init(&b, MM, 1);
    Matrix_Init(x, MM, 1);

    // first
    Matrix Ann;
    ProjectForces(&Ann, AXIS_NORMAL, AXIS_NORMAL);
    Matrix_Put(&A, &Ann, cur_row, 0);
    for (int i = 0; i < N_TANGENTS; i++) {
        Matrix Ant;
        ProjectForces(&Ant, AXIS_NORMAL, AXIS_TANGENT1 + i);
        Matrix_Put(&A, &Ant, cur_row, (i+1)*M);
        Matrix_Negate(&Ant);
        Matrix_Put(&A, &Ant, cur_row, (1 + N_TANGENTS + i)*M);
        Matrix_Free(&Ant);
    }
    Matrix_Free(&Ann);
    cur_row += M;

    // mid
    for (int i = 0; i < N_TANGENTS; i++) {
        Matrix Atn;
        ProjectForces(&Atn, AXIS_TANGENT1 + i, AXIS_NORMAL);
        Matrix_Put(&A, &Atn, cur_row, 0);
        Matrix_Negate(&Atn);
        Matrix_Put(&A, &Atn, cur_row+M, 0);

        int cur_col = M;
        for (int j = 0; j < N_TANGENTS; j++) {
            Matrix Att;
            ProjectForces(&Att, AXIS_TANGENT1 + i, AXIS_TANGENT1 + j);
            
            Matrix_Put(&A, &Att, cur_row, cur_col);
            Matrix_Negate(&Att);
            Matrix_Put(&A, &Att, cur_row, cur_col + M);
            
            Matrix_Put(&A, &Att, cur_row+M, cur_col);
            Matrix_Negate(&Att);
            Matrix_Put(&A, &Att, cur_row+M, cur_col + M);
            Matrix_Free(&Att);
            cur_col += 2*M;
        }

        Matrix_Put(&A, &I, cur_row, MM - M);
        Matrix_Put(&A, &I, cur_row + M, MM - M);
        Matrix_Free(&Atn);
        cur_row += 2*M;
    }

    // last
    for (int i = 0; i < M; i++) {
        for(int j=0; j<M; j++) MATRIX_AT(I, i, j) = 0.0;
        MATRIX_AT(I, i, i) = contacts[i].mu + 1e-6;
    }
    Matrix_Put(&A, &I, cur_row, 0);
    
    for (int i = 0; i < M; i++) {
        for(int j=0; j<M; j++) MATRIX_AT(I, i, j) = (i==j) ? -1.0 : 0.0;
    }
    for (int i = 0; i < 2*N_TANGENTS; i ++)
        Matrix_Put(&A, &I, cur_row, (i+1)*M);
    
    for (int k = 0; k < contacts.size(); k++) {
        Contact *ck = &contacts[k];
        Vec3 rel_accel = Vec3_Sub(bodies[ck->j].accel, bodies[ck->i].accel);
        Vec3 nk = ck->axes[AXIS_NORMAL];
        double jn_a = Vec3_Dot(nk, rel_accel);
        MATRIX_AT(b, k, 0) = jn_a;
        
        cur_row = M;
        for (int i = 0; i < N_TANGENTS; i++) {
            Vec3 tik = ck->axes[AXIS_TANGENT1 + i];
            double jti_a = Vec3_Dot(tik, rel_accel);
            MATRIX_AT(b, cur_row + k, 0) = jti_a;
            MATRIX_AT(b, cur_row + M + k, 0) = -jti_a;
            cur_row += 2*M;
        }
    }

    // printf("A=\n");
    // Matrix_Print(&A);
    // printf("b=\n");
    // Matrix_PrintTransposed(&b);
    
    SolveLemke(&A, &b, x);
    //Verify(&A, &b, x);    

    // printf("x=\n");
    // Matrix_PrintTransposed(x);
    
    Matrix_Free(&A); Matrix_Free(&b);
    Matrix_Free(&I);
}

void Phys_Solve()
{
    Matrix x;
    Solve(&x);
    
    int M = contacts.size();
    for (int i = 0; i < M; i++) {
        Contact &c = contacts[i];
        double fn = MATRIX_AT(x, i, 0);
        c.res_normal_force = fn;

        int cur_row = M;
        for (int j = 0; j < N_TANGENTS; j++) {
            double ftj = MATRIX_AT(x, cur_row + i, 0) - MATRIX_AT(x, cur_row + M + i, 0);
            c.res_tangent_force[j] = ftj;
            cur_row += 2*M;
        }
    }
    Matrix_Free(&x);
}
