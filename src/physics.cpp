#include "physics.h"

#include <cstring>
#include <float.h>
#include "linalg.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "solvers.h"
#include "good_assert.h" 

#define GRAVITY 9.8

std::vector<Body> bodies;
std::vector<Contact> contacts;
int n_tangents = 4;

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

static Vec3 CalcInertia(Body *b)
{
    return Vec3 {
        1/12.0 * b->mass * (b->size.y * b->size.y + b->size.z * b->size.z),
        1/12.0 * b->mass * (b->size.x * b->size.x + b->size.z * b->size.z),
        1/12.0 * b->mass * (b->size.x * b->size.x + b->size.y * b->size.y)
    };
}

Vec3 Phys_ForceEffectOnPoint(Vec3 force_pos, Vec3 center_of_mass, Vec3 force, Vec3 point, double mass, Vec3 inertia)
{
    Vec3 a_lin = Vec3_Scale(force, 1.0 / mass);
    Vec3 torque = Vec3_Cross(Vec3_Sub(force_pos, center_of_mass), force);
    Vec3 a_ang = { torque.x / inertia.x, torque.y / inertia.y, torque.z / inertia.z };
    return Vec3_Add(a_lin, Vec3_Cross(a_ang, Vec3_Sub(point, center_of_mass)));
}

double ProjectForce(int axes, int forces, int k, int l)
{
    Contact *ck = &contacts[k], *cl = &contacts[l];
    Body *bki = &bodies[ck->i], *bkj = &bodies[ck->j];
    double mi = bki->mass, mj = bkj->mass;

    Vec3 project_to = ck->axes[axes];
    
    Vec3 force_i = {0, 0, 0};
    Vec3 force_j = {0, 0, 0};

    if (ck->i == cl->j) force_i = cl->axes[forces];
    if (ck->i == cl->i) force_i = Vec3_Scale(cl->axes[forces], -1.0);
    if (ck->j == cl->j) force_j = cl->axes[forces];
    if (ck->j == cl->i) force_j = Vec3_Scale(cl->axes[forces], -1.0);
    
    Vec3 inertia_i = CalcInertia(bki);
    Vec3 inertia_j = CalcInertia(bkj);

    Vec3 effect_i = Phys_ForceEffectOnPoint(cl->pos, bki->center, force_i, ck->pos, mi, inertia_i);
    Vec3 effect_j = Phys_ForceEffectOnPoint(cl->pos, bkj->center, force_j, ck->pos, mj, inertia_j);
    
    double proj_i = Vec3_Dot(project_to, effect_i);
    double proj_j = Vec3_Dot(project_to, effect_j);

    return proj_j - proj_i;
}

void ProjectForces(Matrix *A, int axes, int forces)
{
    Matrix_Init(A, contacts.size(), contacts.size());
    for (int k = 0; k < contacts.size(); k++) {
        for (int l = 0; l < contacts.size(); l++) {
            MATRIX_AT(*A, k, l) = ProjectForce(axes, forces, k, l);
        }
    }
}

void Phys_SolveLemke()
{
    Vector x;
    int M = contacts.size();
    int MM = (2 + n_tangents*2)*M;
    int cur_row = 0;

    Matrix I;
    Matrix_Init(&I, M, M);
    for (int i = 0; i < M; i++) MATRIX_AT(I, i, i) = 1.0;

    Matrix A;
    Vector b;
    Matrix_Init(&A, MM, MM);
    Vector_Init(&b, MM);
    Vector_Init(&x, MM);

    // first
    Matrix Ann;
    ProjectForces(&Ann, AXIS_NORMAL, AXIS_NORMAL);
    Matrix_Put(&A, &Ann, cur_row, 0);
    for (int i = 0; i < n_tangents; i++) {
        Matrix Ant;
        ProjectForces(&Ant, AXIS_NORMAL, AXIS_TANGENT1 + i);
        Matrix_Put(&A, &Ant, cur_row, (2*i+1)*M);
        Matrix_Negate(&Ant);
        Matrix_Put(&A, &Ant, cur_row, (2*i+2)*M);
        Matrix_Free(&Ant);
    }
    Matrix_Free(&Ann);
    cur_row += M;

    // mid
    for (int i = 0; i < n_tangents; i++) {
        Matrix Atn;
        ProjectForces(&Atn, AXIS_TANGENT1 + i, AXIS_NORMAL);
        Matrix_Put(&A, &Atn, cur_row, 0);
        Matrix_Negate(&Atn);
        Matrix_Put(&A, &Atn, cur_row+M, 0);

        int cur_col = M;
        for (int j = 0; j < n_tangents; j++) {
            Matrix Att;
            ProjectForces(&Att, AXIS_TANGENT1 + i, AXIS_TANGENT1 + j);
            
            Matrix_Put(&A, &Att, cur_row, cur_col);
            Matrix_Put(&A, &Att, cur_row + M, cur_col + M);
            Matrix_Negate(&Att);
            Matrix_Put(&A, &Att, cur_row + M, cur_col);
            Matrix_Put(&A, &Att, cur_row, cur_col + M);

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
    for (int i = 0; i < 2*n_tangents; i ++)
        Matrix_Put(&A, &I, cur_row, (i+1)*M);
    
    for (int k = 0; k < contacts.size(); k++) {
        Contact *ck = &contacts[k];
        Body *bi = &bodies[ck->i];
        Body *bj = &bodies[ck->j];

        Vec3 inertia_i = CalcInertia(bi);
        Vec3 inertia_j = CalcInertia(bj);

        Vec3 i_effect = Phys_ForceEffectOnPoint(bi->center, bi->center, bi->force, ck->pos, bi->mass, inertia_i);
        Vec3 j_effect = Phys_ForceEffectOnPoint(bj->center, bj->center, bj->force, ck->pos, bj->mass, inertia_j);
        
        Vec3 rel_accel = Vec3_Sub(j_effect, i_effect);
        VECTOR_AT(b, k) = Vec3_Dot(ck->axes[AXIS_NORMAL], rel_accel);
        
        cur_row = M;
        for (int i = 0; i < n_tangents; i++) {
            Vec3 tik = ck->axes[AXIS_TANGENT1 + i];
            double jti_a = Vec3_Dot(tik, rel_accel);
            VECTOR_AT(b, cur_row + k) = jti_a;
            VECTOR_AT(b, cur_row + M + k) = -jti_a;
            cur_row += 2*M;
        }
    }
    
    for (int i = 0; i < A.rows; i++) {
        MATRIX_AT(A, i, i) += 1e-6;
    }

    SolveLemke(&A, &b, &x);
    
    for (int i = 0; i < M; i++) {
        Contact &c = contacts[i];
        double fn = VECTOR_AT(x, i);
        c.res_normal_force = fn;

        int cur_row = M;
        for (int j = 0; j < n_tangents; j++) {
            double ftj = VECTOR_AT(x, cur_row + i) - VECTOR_AT(x, cur_row + M + i);
            c.res_tangent_force[j] = ftj;
            cur_row += 2*M;
        }
    }

    Matrix_Free(&A); Vector_Free(&b);
    Matrix_Free(&I);
    Vector_Free(&x);
}

static void Project(Vector *x)
{
    int M = contacts.size();
    for (int i = 0; i < M; i++) {
        VECTOR_AT(*x, i) = std::max(0.0, VECTOR_AT(*x, i));
        
        double n = VECTOR_AT(*x, i);
        double u = VECTOR_AT(*x, i + M);
        double v = VECTOR_AT(*x, i + 2*M);

        double len = sqrt(u*u + v*v);
        double lim = n * contacts[i].mu;
        if (len > lim) {
            VECTOR_AT(*x, i + M) = u / len * lim;
            VECTOR_AT(*x, i + 2*M) = v / len * lim;
        }
    }
}

void Phys_SolveProjected(SolveProjectedFunc solve_func)
{
    int M = contacts.size();
    int cur_row = 0;
    
    Matrix A;
    Vector b, x;
    Matrix_Init(&A, 3*M, 3*M);
    Vector_Init(&b, 3*M);
    Vector_Init(&x, 3*M);

    // mid
    for (int i = 0; i < n_tangents+1; i++) {
        for (int j = 0; j < n_tangents+1; j++) {
            Matrix Aij;
            ProjectForces(&Aij, i, j);
            Matrix_Put(&A, &Aij, i*M, j*M);
            Matrix_Free(&Aij);
        }
    }

    for (int k = 0; k < contacts.size(); k++) {
        Contact *ck = &contacts[k];
        Vec3 rel_accel = Vec3_Sub(
            Vec3_Scale(bodies[ck->j].force, 1.0 / bodies[ck->j].mass),
            Vec3_Scale(bodies[ck->i].force, 1.0 / bodies[ck->i].mass));
        
        Vec3 n = ck->axes[AXIS_NORMAL];
        Vec3 u = ck->axes[AXIS_TANGENT1];
        Vec3 v = ck->axes[AXIS_TANGENT2];
        
        double rel_n = Vec3_Dot(n, rel_accel);
        double rel_u = Vec3_Dot(u, rel_accel);
        double rel_v = Vec3_Dot(v, rel_accel);
        
        VECTOR_AT(b, k) = -rel_n;
        VECTOR_AT(b, k + M) = -rel_u;
        VECTOR_AT(b, k + 2*M) = -rel_v;
    }

    solve_func(&A, &b, &x, Project);
    
    for (int i = 0; i < M; i++) {
        Contact &c = contacts[i];
        c.res_normal_force     = VECTOR_AT(x, i);
        c.res_tangent_force[0] = VECTOR_AT(x, i + M);
        c.res_tangent_force[1] = VECTOR_AT(x, i + 2*M);
    }

    Matrix_Free(&A);
    Vector_Free(&b);
    Vector_Free(&x);
}

struct {
    Matrix A;
    Vector b, x;

    std::vector<int> best_indices;
    double best_err;
    
    std::vector<int> indices;
} enum_data;

Vec3 Phys_GetBodyAccelEx(int i, Vec3 point, Vector &forces)
{
    int M = contacts.size();
    Body* body = &bodies[i];
    Vec3 inertia = CalcInertia(body);
    Vec3 res = Phys_ForceEffectOnPoint(body->center, body->center, body->force, point, body->mass, inertia);

    for (int j = 0; j < M; j++) {
        Contact &c = contacts[j];
        Vec3 n_force = {0, 0, 0};
        if (c.j == i)      n_force = Vec3_Scale(c.axes[AXIS_NORMAL], VECTOR_AT(forces, j));
        else if (c.i == i) n_force = Vec3_Scale(c.axes[AXIS_NORMAL], -VECTOR_AT(forces, j));
        Vec3 n_effect = Phys_ForceEffectOnPoint(c.pos, body->center, n_force, point, body->mass, inertia);
        res = Vec3_Add(res, n_effect);

        Vec3 t_force = {0, 0, 0};
        if (c.j == i)      t_force = Vec3_Scale(c.axes[AXIS_TANGENT1 + enum_data.indices[j]], VECTOR_AT(forces, j + M) - VECTOR_AT(forces, j + 2*M));
        else if (c.i == i) t_force = Vec3_Scale(c.axes[AXIS_TANGENT1 + enum_data.indices[j]], -VECTOR_AT(forces, j + M) + VECTOR_AT(forces, j + 2*M));
        
        Vec3 t_effect = Phys_ForceEffectOnPoint(c.pos, body->center, t_force, point, body->mass, inertia);
        res = Vec3_Add(res, t_effect);
    }
    
    return res;
}

static void Solve_ForIndices()
{
    Vector x;
    int M = contacts.size();
 
    for (int k = 0; k < contacts.size(); k++) {
        for (int l = 0; l < contacts.size(); l++) {
            double nn = ProjectForce(AXIS_NORMAL, AXIS_NORMAL, k, l);
            double nt = ProjectForce(AXIS_NORMAL, AXIS_TANGENT1 + enum_data.indices[l], k, l);
            double tn = ProjectForce(AXIS_TANGENT1 + enum_data.indices[k], AXIS_NORMAL, k, l);
            double tt = ProjectForce(AXIS_TANGENT1 + enum_data.indices[k], AXIS_TANGENT1 + enum_data.indices[l], k, l);
            
            MATRIX_AT(enum_data.A, k, l) = nn;
            MATRIX_AT(enum_data.A, k, l + M) = nt;
            MATRIX_AT(enum_data.A, k, l + 2*M) = -nt;

            MATRIX_AT(enum_data.A, k + M, l) = tn;
            MATRIX_AT(enum_data.A, k + 2*M, l) = -tn;

            MATRIX_AT(enum_data.A, k + M, l + M) = tt;
            MATRIX_AT(enum_data.A, k + 2*M, l + 2*M) = tt;
            MATRIX_AT(enum_data.A, k + 2*M, l + M) = -tt;
            MATRIX_AT(enum_data.A, k + M, l + 2*M) = -tt;
        }
    }

    for (int i = 0; i < M; i++) {
        MATRIX_AT(enum_data.A, M + i, 3*M + i) = 1.0;
        MATRIX_AT(enum_data.A, 2*M + i, 3*M + i) = 1.0;

        MATRIX_AT(enum_data.A, 3*M + i, i) = contacts[i].mu + 1e-6;
        MATRIX_AT(enum_data.A, 3*M + i, M + i) = -1.0;
        MATRIX_AT(enum_data.A, 3*M + i, 2*M + i) = -1.0;
    }
    
    for (int k = 0; k < contacts.size(); k++) {
        Contact *ck = &contacts[k];
        Body *bi = &bodies[ck->i];
        Body *bj = &bodies[ck->j];

        Vec3 inertia_i = CalcInertia(bi);
        Vec3 inertia_j = CalcInertia(bj);

        Vec3 i_effect = Phys_ForceEffectOnPoint(bi->center, bi->center, bi->force, ck->pos, bi->mass, inertia_i);
        Vec3 j_effect = Phys_ForceEffectOnPoint(bj->center, bj->center, bj->force, ck->pos, bj->mass, inertia_j);
        
        Vec3 rel_accel = Vec3_Sub(j_effect, i_effect);
        VECTOR_AT(enum_data.b, k) = Vec3_Dot(ck->axes[AXIS_NORMAL], rel_accel);
        
        Vec3 tik = ck->axes[AXIS_TANGENT1 + enum_data.indices[k]];
        double jti_a = Vec3_Dot(tik, rel_accel);
        VECTOR_AT(enum_data.b, M + k) = jti_a;
        VECTOR_AT(enum_data.b, 2*M + k) = -jti_a;
    }
    
    for (int i = 0; i < enum_data.A.rows; i++) {
        MATRIX_AT(enum_data.A, i, i) += 1e-6;
    }

    SolveLemke(&enum_data.A, &enum_data.b, &enum_data.x);

    // { mu * f_n - |f_t| >= 0
    // { |a_t| (mu * f_n - |f_t|) = 0
    // { |a_t| |f_t| + a_t * f_t = 0
    
    double err = 0.0;
    for (int i = 0; i < M; i++) {
        Contact &c = contacts[i];
        Vec3 a_i = Phys_GetBodyAccelEx(c.i, c.pos, enum_data.x);
        Vec3 a_j = Phys_GetBodyAccelEx(c.j, c.pos, enum_data.x);
        Vec3 a = Vec3_Sub(a_j, a_i);
        
        Vec3 normal = c.axes[AXIS_NORMAL];
        Vec3 t_basis1 = c.axes[AXIS_TANGENT1 + enum_data.indices[i]];
        Vec3 t_basis2 = Vec3_Cross(normal, t_basis1);
        
        double ft_plus = VECTOR_AT(enum_data.x, i + M);
        double ft_minus = VECTOR_AT(enum_data.x, i + 2*M);

        Vec2 at = { Vec3_Dot(a, t_basis1), Vec3_Dot(a, t_basis2) };
        Vec2 ft = { ft_plus - ft_minus, 0 };
        
        double ft_len = ft_plus + ft_minus;
        double fn_len = VECTOR_AT(enum_data.x, i);
        double at_len = Vec2_Length(at);
        double an_len = Vec3_Dot(a, normal);

        // err += -std::min(0.0, fn_len);
        // err += -std::min(0.0, an_len);
        // err += fabs(fn_len * an_len);
        // err += -std::max(0.0, c.mu * fn_len - ft_len);
        err += fabs(at_len * (c.mu * fn_len - ft_len));
        err += fabs(at_len * ft_len + Vec2_Dot(at, ft));
    }

    if (err < enum_data.best_err) {
        enum_data.best_err = err;
        enum_data.best_indices = enum_data.indices;

        for (int i = 0; i < M; i++) {
            Contact &c = contacts[i];
            memset(c.res_tangent_force, 0, sizeof(c.res_tangent_force));
            c.res_normal_force = VECTOR_AT(enum_data.x, i);
            c.res_tangent_force[enum_data.indices[i]] = VECTOR_AT(enum_data.x, i + M) - VECTOR_AT(enum_data.x, i + 2*M);
        }
    }
}

static void Solve_ForAllIndices(int pos)
{
    if (pos == enum_data.indices.size())
    {
        int M = contacts.size();
        Solve_ForIndices();
        return;
    }

    for (int i = 0; i < n_tangents; i++) {
        enum_data.indices[pos] = i;
        if (pos == 4) {
            printf("%d %d %d %d %d: cur_err=%0.015lf\n",
                enum_data.indices[0],
                enum_data.indices[1],
                enum_data.indices[2],
                enum_data.indices[3],
                enum_data.indices[4],
                enum_data.best_err
            );
        }
        Solve_ForAllIndices(pos + 1);
    }
}

void Phys_SolveEnum()
{
    int M = contacts.size();
    enum_data.best_err = DBL_MAX;
    enum_data.best_indices.resize(M);
    enum_data.indices.resize(M);
    Matrix_Init(&enum_data.A, 4*M, 4*M);
    Vector_Init(&enum_data.b, 4*M);
    Vector_Init(&enum_data.x, 4*M);

    
#ifdef ENUM_RND
    for (int i = 0; i < ENUM_RND; i++) {
        for (int j = 0; j < M; j++) {
            enum_data.indices[j] = rand() % N_TANGENTS;
        }
        Solve_ForIndices();
        
        if ((i % 50) == 0) {
            printf("%d %lf\n", i, enum_data.best_err);
            fprintf(stderr, "%d %lf\n", i, enum_data.best_err);
        }
    }
#else
    Solve_ForAllIndices(0);
#endif

    enum_data.indices = enum_data.best_indices;
    Solve_ForIndices();
    
    Matrix_Print(&enum_data.A);
    printf("\n\n");
    Vector_Print(&enum_data.b);
    printf("\n\n");
    Vector_Print(&enum_data.x);

    //printf("Best indices: ");
    //for (int i = 0; i < M; i++) printf("%d ", enum_data.best_indices[i]);
    //printf("\n");

    Matrix_Free(&enum_data.A);
    Vector_Free(&enum_data.b);
    Vector_Free(&enum_data.x);
}

void Phys_TangentsFromEuler()
{
    for (Contact &c : contacts) {
        float cx = cos(c.angles.x * M_PI / 180), sx = sin(c.angles.x * M_PI / 180);
        float cy = cos(c.angles.y * M_PI / 180), sy = sin(c.angles.y * M_PI / 180);
        float cz = cos(c.angles.z * M_PI / 180), sz = sin(c.angles.z * M_PI / 180);
        c.axes[AXIS_NORMAL].x = sx*sy*cz - cx*sz;
        c.axes[AXIS_NORMAL].y = sx*sy*sz + cx*cz;
        c.axes[AXIS_NORMAL].z = sx*cy;

        Vec3 T0 = {cy*cz, cy*sz, -sy};
        Vec3 T1 = Vec3_Cross(T0, c.axes[AXIS_NORMAL]);
        
        for (int i = 0; i < n_tangents; i++) {
            double alpha = i * M_PI / (double)n_tangents;
            double cs = cos(alpha), sn = sin(alpha);
            c.axes[AXIS_TANGENT1 + i] = Vec3_Add(Vec3_Scale(T0, cs), Vec3_Scale(T1, sn));
        }

        for (int i = 0; i < n_tangents+1; i++) {
            c.axes[i] = Vec3_Normalize(c.axes[i]);
        }
    }
}

Vec3 Phys_GetBodyAccel(int i, Vec3 point)
{
    Body* body = &bodies[i];
    Vec3 inertia = CalcInertia(body);
    Vec3 res = Phys_ForceEffectOnPoint(body->center, body->center, body->force, point, body->mass, inertia);

    for (int j = 0; j < contacts.size(); j++) {
        Contact &c = contacts[j];
        Vec3 n_force = {0, 0, 0};
        if (c.j == i)      n_force = Vec3_Scale(c.axes[AXIS_NORMAL], c.res_normal_force);
        else if (c.i == i) n_force = Vec3_Scale(c.axes[AXIS_NORMAL], -c.res_normal_force);
        Vec3 n_effect = Phys_ForceEffectOnPoint(c.pos, body->center, n_force, point, body->mass, inertia);
        res = Vec3_Add(res, n_effect);

        for (int k = 0; k < n_tangents; k++) {
            Vec3 t_force = {0, 0, 0};
            if (c.j == i)      t_force = Vec3_Scale(c.axes[AXIS_TANGENT1 + k], c.res_tangent_force[k]);
            else if (c.i == i) t_force = Vec3_Scale(c.axes[AXIS_TANGENT1 + k], -c.res_tangent_force[k]);
            
            Vec3 t_effect = Phys_ForceEffectOnPoint(c.pos, body->center, t_force, point, body->mass, inertia);
            res = Vec3_Add(res, t_effect);
        }
    }
    
    return res;
}