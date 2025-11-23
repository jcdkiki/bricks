#include "physics.h"

#include <float.h>
#include <glad/glad.h>
#include <imgui.h>

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
    fwrite(contacts.data(), sizeof(Contact), n_contacts, f);
    
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
    fread(contacts.data(), sizeof(Contact), n_contacts, f);

    for (int i = 0; i < n_contacts; i++) {
        contacts[i].normal_force = 0.0;
        contacts[i].tangent_force = 0.0;
    }
}

enum {
    AXIS_NORMAL,
    AXIS_TANGENT
};

void ProjectForces(Matrix *A, int forces, int axes)
{
    for (int k = 0; k < contacts.size(); k++) {
        for (int l = 0; l < contacts.size(); l++) {
            Contact *ck = &contacts[k], *cl = &contacts[l];
            Body *bki = &bodies[ck->i], *bkj = &bodies[ck->j];
            double mi = bki->mass, mj = bkj->mass;
            
            Vec2 force_axis = (forces == AXIS_NORMAL) ? cl->normal : cl->tangent;
            Vec2 project_to = (axes == AXIS_NORMAL) ? ck->normal : ck->tangent;
            double nk_nl = Vec2_Dot(force_axis, project_to);

            if (l == k) {
                MATRIX_AT(*A, k, l) = nk_nl / mi + nk_nl / mj;
            }
            else if (cl->j == ck->j) MATRIX_AT(*A, k, l) = nk_nl / mj;
            else if (cl->i == ck->i) MATRIX_AT(*A, k, l) = nk_nl / mi;
            else if (cl->i == ck->j) MATRIX_AT(*A, k, l) = -nk_nl / mj;
            else if (cl->j == ck->i) MATRIX_AT(*A, k, l) = -nk_nl / mi;
        }
    }
}

static void Solve(Matrix *x)
{    
    int M = contacts.size();

    Matrix Ann, Ant, Atn, Att;
    Matrix_Init(&Ann, M, M);
    Matrix_Init(&Ant, M, M);
    Matrix_Init(&Atn, M, M);
    Matrix_Init(&Att, M, M);

    ProjectForces(&Ann, AXIS_NORMAL, AXIS_NORMAL);
    ProjectForces(&Ant, AXIS_NORMAL, AXIS_TANGENT);
    ProjectForces(&Atn, AXIS_TANGENT, AXIS_NORMAL);
    ProjectForces(&Att, AXIS_TANGENT, AXIS_TANGENT);

    Matrix A, b;
    Matrix_Init(&A, M * 4, M * 4);
    Matrix_Init(&b, M * 4, 1);
    Matrix_Init(x, M * 4, 1);

    // Ann Ant -Ant 0
    Matrix_Put(&A, &Ann, 0, 0);
    Matrix_Put(&A, &Ant, 0, M);
    for(int i=0; i<M*M; i++) Ant.data[i] *= -1.0;
    Matrix_Put(&A, &Ant, 0, 2*M);
    for(int i=0; i<M*M; i++) Ant.data[i] *= -1.0;

    // Atn Att -Att
    Matrix_Put(&A, &Atn, M, 0);
    Matrix_Put(&A, &Att, M, M);
    for(int i=0; i<M*M; i++) Att.data[i] *= -1.0;
    Matrix_Put(&A, &Att, M, 2*M);
    for(int i=0; i<M*M; i++) Att.data[i] *= -1.0;

    // -Atn -Att Att
    for(int i=0; i<M*M; i++) Atn.data[i] *= -1.0;
    Matrix_Put(&A, &Atn, 2*M, 0);
    for(int i=0; i<M*M; i++) Att.data[i] *= -1.0;
    Matrix_Put(&A, &Att, 2*M, M);
    for(int i=0; i<M*M; i++) Att.data[i] *= -1.0;
    Matrix_Put(&A, &Att, 2*M, 2*M);

    Matrix I;
    Matrix_Init(&I, M, M);
    for (int i = 0; i < M; i++) MATRIX_AT(I, i, i) = 1.0;
    Matrix_Put(&A, &I, M, 3*M);
    Matrix_Put(&A, &I, 2*M, 3*M);

    for (int i = 0; i < M; i++) {
        for(int j=0; j<M; j++) MATRIX_AT(I, i, j) = 0.0;
        MATRIX_AT(I, i, i) = contacts[i].mu;
    }
    Matrix_Put(&A, &I, 3*M, 0);

    for (int i = 0; i < M; i++) {
        for(int j=0; j<M; j++) MATRIX_AT(I, i, j) = (i==j) ? -1.0 : 0.0;
    }
    Matrix_Put(&A, &I, 3*M, 1*M);
    Matrix_Put(&A, &I, 3*M, 2*M);

    
    for (int k = 0; k < contacts.size(); k++) {
        Contact *ck = &contacts[k];
        Vec2 nk = ck->normal;
        Vec2 tk = ck->tangent;
        Vec2 rel_accel = Vec2_Sub(bodies[ck->j].accel, bodies[ck->i].accel);

        double jn_a = Vec2_Dot(nk, rel_accel);
        double jt_a = Vec2_Dot(tk, rel_accel);

        MATRIX_AT(b, k, 0)       = jn_a;
        MATRIX_AT(b, k + M, 0)   = jt_a;
        MATRIX_AT(b, k + 2*M, 0) = -jt_a; 
        MATRIX_AT(b, k + 3*M, 0) = 0;
    }

    SolveLemke(&A, &b, x);
    
    Matrix_Free(&A); Matrix_Free(&b);
    Matrix_Free(&Ann); Matrix_Free(&Ant); Matrix_Free(&Atn); Matrix_Free(&Att);
    Matrix_Free(&I);
}

void Phys_Solve()
{
    Matrix x;
    Solve(&x);
    
    for (int i = 0; i < contacts.size(); i++) {
        double fn = MATRIX_AT(x, i, 0);
        double ft_plus = MATRIX_AT(x, i + contacts.size(), 0);
        double ft_minus = MATRIX_AT(x, i + 2*contacts.size(), 0);

        contacts[i].normal_force = fn;
        contacts[i].tangent_force = ft_plus - ft_minus;
    }
    Matrix_Free(&x);
}
