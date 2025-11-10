#include "physics.h"

#include <float.h>
#include <glad/glad.h>
#include "text.h"

#include "linalg.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define GRAVITY 9.8

#define N_BODIES 3
#define N_CONTACTS 2

enum {
    MODE_INPUT,
    MODE_OUTPUT,
    MODE_TOTAL
};

struct {
    bool show_ids;
    int mode;
    float line_len;
    int show_axes;
} settings = { 1, MODE_INPUT, 4.f, 1 };

typedef struct {
    int i1, i2;
    Vec2 normal;
    Vec2 tangent;
    Vec2 rel_accel;
    double mu;

    // just for visuals
    Vec2 pos;

    // solution
    double normal_force;
    double tangent_force;
} Contact;

typedef struct {
    double mass;
    Vec2 accel;

    // just for visuals
    Vec2 center;
    Vec2 size;
    double angle;
} Body;

Body bodies[N_BODIES];
Contact contacts[N_CONTACTS];

static void ProjectedGaussSeidel(Matrix *A, Matrix *b, Matrix *x)
{
    for (int iter = 0; iter < 1000; iter++) {
        for (int i = 0; i < x->rows; i++) {
            double new_xi = MATRIX_AT(*b, i, 0);
            for (int j = 0; j < x->rows; j++) {
                if (i == j) continue;
                new_xi -= MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
            }

            new_xi /= MATRIX_AT(*A, i, i);
            MATRIX_AT(*x, i, 0) = new_xi;
        }

        for (int i = 0; i < x->rows; i++) {
            double x_i = MATRIX_AT(*x, i, 0);
            MATRIX_AT(*x, i, 0) = fmax(0.f, x_i);
        }
    }
}

enum {
    AXIS_NORMAL,
    AXIS_TANGENT
};

static void InitInvMassMatrix(Matrix *M)
{
    Matrix_Init(M, N_BODIES * 2, N_BODIES * 2);

    for (int i = 0; i < N_BODIES; i++) {
        double inv_mass = 1.f / bodies[i].mass;
        MATRIX_AT(*M, i*2, i*2) = inv_mass;
        MATRIX_AT(*M, i*2 + 1, i*2 + 1) = inv_mass;
    }
}

static void InitMuMatrix(Matrix *Mu)
{
    Matrix_Init(Mu, N_CONTACTS, N_CONTACTS);
    for (int i = 0; i < N_CONTACTS; i++) {
        Contact *c = &contacts[i];
        MATRIX_AT(*Mu, i, i) = c->mu;
    }
}

static void Solve(Matrix *x)
{
    int N = N_BODIES;
    int M = N_CONTACTS;

    Matrix Mass;
    InitInvMassMatrix(&Mass);
    printf("Mass = \n"); Matrix_Print(&Mass);
    
    Matrix Mu;
    InitMuMatrix(&Mu);

    // E = 
    // [ 1 1 0 0 0 0 ]
    // [ 0 0 ... 0 0 ]
    // [ 0 0 0 0 1 1 ]
    Matrix E;
    Matrix_Init(&E, M, 2*M);
    for (int i = 0; i < M; i++) {
        MATRIX_AT(E, i, i*2 + 0) = 1;
        MATRIX_AT(E, i, i*2 + 1) = 1;
    }
    printf("E = \n"); Matrix_Print(&E);

    Matrix negET;
    Matrix_InitTransposed(&E, &negET);
    for (int i = 0; i < negET.rows * negET.cols; i++) {
        negET.data[i] *= -1;
    }

    // C = 
    // [ 0   0   0 ]
    // [ 0   0   E ]
    // [ Mu -E^T 0 ]
    Matrix C;
    Matrix_Init(&C, 4*M, 4*M);
    Matrix_Put(&C, &Mu, 3*M, 0);
    Matrix_Put(&C, &negET, 2*M, M);
    Matrix_Put(&C, &E, M, 2*M);
    printf("C = \n"); Matrix_Print(&C);

    Matrix Jn;
    Matrix_Init(&Jn, N_CONTACTS, N_BODIES * 2);
    for (int i = 0; i < N_CONTACTS; i++) {
        Contact *c = &contacts[i];
        MATRIX_AT(Jn, i, c->i1*2 + 0) = c->normal.x;
        MATRIX_AT(Jn, i, c->i1*2 + 1) = c->normal.y;
        MATRIX_AT(Jn, i, c->i2*2 + 0) = -c->normal.x;
        MATRIX_AT(Jn, i, c->i2*2 + 1) = -c->normal.y;
    }
    printf("Jn = \n"); Matrix_Print(&Jn);

    Matrix Jt;
    Matrix_Init(&Jt, N_CONTACTS * 2, N_BODIES * 2);
    for (int i = 0; i < N_CONTACTS; i++) {
        Contact *c = &contacts[i];
        MATRIX_AT(Jt, i, c->i1*2 + 0) = c->tangent.x;
        MATRIX_AT(Jt, i, c->i1*2 + 1) = c->tangent.y;
        MATRIX_AT(Jt, i, c->i2*2 + 0) = -c->tangent.x;
        MATRIX_AT(Jt, i, c->i2*2 + 1) = -c->tangent.y;

        MATRIX_AT(Jt, i + N_CONTACTS, c->i1*2 + 0) = -c->tangent.x;
        MATRIX_AT(Jt, i + N_CONTACTS, c->i1*2 + 1) = -c->tangent.y;
        MATRIX_AT(Jt, i + N_CONTACTS, c->i2*2 + 0) = c->tangent.x;
        MATRIX_AT(Jt, i + N_CONTACTS, c->i2*2 + 1) = c->tangent.y;
    }
    printf("Jt = \n"); Matrix_Print(&Jt);

    // G = 
    // [J_t]
    // [J_n]
    // [0]
    Matrix G;
    Matrix_Init(&G, 4*M, 2*N);
    Matrix_Put(&G, &Jt, 0, 0);
    Matrix_Put(&G, &Jn, N_CONTACTS*2, 0);
    printf("G = \n"); Matrix_Print(&G);

    Matrix GT;
    Matrix_InitTransposed(&G, &GT);

    // A = C + G * M * G^T
    Matrix GM;
    Matrix_Init(&GM, 4*M, 2*N);
    Matrix_Mul(&G, &Mass, &GM);
    
    Matrix GMGT;
    Matrix_Init(&GMGT, 4*M, 4*M);
    Matrix_Mul(&GM, &GT, &GMGT);

    Matrix A;
    Matrix_Init(&A, 4*M, 4*M);
    Matrix_Add(&C, &GMGT, &A);

    // b = G * M * F (all forces)
    Matrix F;
    Matrix_Init(&F, 2*N, 1);
    for (int i = 0; i < N_BODIES; i++) {
        Body *b = &bodies[i];
        MATRIX_AT(F, i*2, 0) = b->accel.x;
        MATRIX_AT(F, i*2 + 1, 0) = b->accel.y;
    }
    printf("F = \n"); Matrix_Print(&F);
    
    Matrix b;
    Matrix_Init(&b, 4*M, 1);
    Matrix_Mul(&GM, &F, &b);
    // for (int i = 0; i < b.rows * b.cols; i++) {
    //     b.data[i] *= -1.0;
    // }

    // for (int i = 0; i < A.rows; i++) {
    // if (fabs(A.data[i * A.cols + i]) < 1e-6) {
    //     A.data[i * A.cols + i] = 1e-6;
    // }

    // x = [f_n  f_t^+  f_t^-  beta]^T
    Matrix_Init(x, 4*M, 1);
    ProjectedGaussSeidel(&A, &b, x);

    printf("A = \n");
    Matrix_Print(&A);
    
    printf("b = \n");
    Matrix_Print(&b);
    
    printf("x = \n");
    Matrix_Print(x);

    Matrix_Free(&A); Matrix_Free(&b); Matrix_Free(&F);
    Matrix_Free(&GMGT); Matrix_Free(&GM); Matrix_Free(&GT); Matrix_Free(&G);
    Matrix_Free(&Jt); Matrix_Free(&Jn); Matrix_Free(&C);
    Matrix_Free(&negET); Matrix_Free(&E); Matrix_Free(&Mu); Matrix_Free(&Mass);
}

void Phys_Init()
{
    double theta = 30.f / 180.f * M_PI;
    double c_theta = cos(theta);
    double s_theta = sin(theta);

    bodies[0] = (Body) {
        .mass = INFINITY,
        .accel = {0, 0},

        .size = {1000, 50},
        .center = {500, 500},
        .angle = theta
    };
    
    bodies[1] = (Body) {
        .mass = 1.f,
        .accel = {0, GRAVITY},

        .size = {100, 50},
        .center = {500 + 50 * s_theta, 500 - 50 * c_theta},
        .angle = theta
    };

    bodies[2] = (Body) {
        .mass = 1.f,
        .accel = {0, GRAVITY},

        .size = {100, 50},
        .center = {500 + 100 * s_theta, 500 - 100 * c_theta},
        .angle = theta
    };

    contacts[0] = (Contact) {
        .i1 = 0,
        .i2 = 1,
        .normal = {s_theta, -c_theta},
        .tangent = {-c_theta, -s_theta},
        .rel_accel = Vec2_Sub(bodies[1].accel, bodies[0].accel),
        .mu = 1.f,

        .pos = {500 + 25 * s_theta, 500 - 25 * c_theta}
    };

    contacts[1] = (Contact) {
        .i1 = 1,
        .i2 = 2,
        .normal = {s_theta, -c_theta},
        .tangent = {-c_theta, -s_theta},
        .rel_accel = Vec2_Sub(bodies[2].accel, bodies[1].accel),
        .mu = 1.f,

        .pos = {500 + 75 * s_theta, 500 - 75 * c_theta}
    };

    Matrix x;
    Solve(&x);

    for (int i = 0; i < N_CONTACTS; i++) {
        contacts[i].normal_force = MATRIX_AT(x, i, 0);
        contacts[i].tangent_force = MATRIX_AT(x, i + N_CONTACTS, 0)
                                  - MATRIX_AT(x, i + 2*N_CONTACTS, 0);
    }
    
    Matrix_Free(&x);
}

void DrawArrow(Vec2 p1, Vec2 p2)
{
    glLineWidth(3.f);
    glBegin(GL_LINES);
    glVertex2f(p1.x, p1.y);
    glVertex2f(p2.x, p2.y);
    
    Vec2 dir = Vec2_Normalize(Vec2_Sub(p1, p2));
    Vec2 perp = {-dir.y, dir.x};
    Vec2 tip1 = Vec2_Add(p2, Vec2_Scale(Vec2_Add(dir, perp), 5.f));
    Vec2 tip2 = Vec2_Add(p2, Vec2_Scale(Vec2_Sub(dir, perp), 5.f));
    
    glVertex2f(p2.x, p2.y);
    glVertex2f(tip1.x, tip1.y);
    glVertex2f(p2.x, p2.y);
    glVertex2f(tip2.x, tip2.y);
    glEnd();
}

void Phys_Draw()
{
    glDisable(GL_DEPTH_TEST);
    
    glColor3f(1.f, 1.f, 1.f);
    glLineWidth(1.f);
    for (int i = 0; i < N_BODIES; i++) {
        glPushMatrix();
        glTranslated(bodies[i].center.x, bodies[i].center.y, 0);
        glRotated(bodies[i].angle * 180.f / M_PI, 0, 0, 1);
        glBegin(GL_LINE_LOOP);
        glVertex2d(bodies[i].size.x / 2, bodies[i].size.y / 2);
        glVertex2d(bodies[i].size.x / 2, -bodies[i].size.y / 2);
        glVertex2d(-bodies[i].size.x / 2, -bodies[i].size.y / 2);
        glVertex2d(-bodies[i].size.x / 2, bodies[i].size.y / 2);
        glEnd();
        glPopMatrix();
    }

    if (settings.show_ids) {
        for (int i = 0; i < N_BODIES; i++) {
            char buf[8];
            sprintf(buf, "%d", i);
            Text_Draw(bodies[i].center.x - 8.f, bodies[i].center.y - 8.f, buf, 16.f);
        }
    }

    if (settings.mode == MODE_INPUT) {
        // accel
        glColor3f(1.f, 0.3f, 0.3f);
        for (int i = 0; i < N_BODIES; i++) {
            Body *b = &bodies[i];
            if (b->accel.x == 0 && b->accel.y == 0) continue;

            Vec2 p = Vec2_Add(b->center, Vec2_Scale(b->accel, settings.line_len));
            DrawArrow(b->center, p);
        }
    
        // rel_accel
        glColor3f(0.3f, 1.f, 0.3f);
        for (int i = 0; i < N_CONTACTS; i++) {
            Contact *c = &contacts[i];
            if (c->rel_accel.x == 0 && c->rel_accel.y == 0) continue;

            Vec2 p = Vec2_Add(c->pos, Vec2_Scale(c->rel_accel, settings.line_len));
            DrawArrow(c->pos, p);

            double n_scale = Vec2_Dot(c->normal, c->rel_accel) * settings.line_len;
            double t_scale = Vec2_Dot(c->tangent, c->rel_accel) * settings.line_len;

            Vec2 p1 = Vec2_Add(c->pos, Vec2_Scale(c->normal, n_scale));
            Vec2 p2 = Vec2_Add(c->pos, Vec2_Scale(c->tangent, t_scale));
            if (n_scale != 0) DrawArrow(c->pos, p1);
            if (t_scale != 0) DrawArrow(c->pos, p2);

        }
    
        // contact points
        glColor3f(1.f, 1.f, 0.3f);
        glPointSize(5.f);
        glBegin(GL_POINTS);
        for (int i = 0; i < N_CONTACTS; i++) {
            glVertex2f(contacts[i].pos.x, contacts[i].pos.y);
        }
        glEnd();
    }

    if (settings.mode == MODE_OUTPUT) {
        glColor3f(1.f, 1.f, 0.3f);
        for (int i = 0; i < N_CONTACTS; i++) {
            Contact *c = &contacts[i];
            Vec2 p1 = Vec2_Add(c->pos, Vec2_Scale(c->normal, c->normal_force * settings.line_len));
            Vec2 p2 = Vec2_Add(c->pos, Vec2_Scale(c->tangent, c->tangent_force * settings.line_len));
            if (c->normal_force != 0) DrawArrow(c->pos, p1);
            if (c->tangent_force != 0) DrawArrow(c->pos, p2);
        }
    }

    if (settings.mode == MODE_TOTAL) {
        for (int i = 0; i < N_BODIES; i++) {
            Body *b = &bodies[i];
            Vec2 accel = b->accel;

            for (int j = 0; j < N_CONTACTS; j++) {
                Contact *c = &contacts[j];
                if (c->i2 == i) {
                    accel = Vec2_Add(accel, Vec2_Scale(c->normal, c->normal_force / b->mass));
                    accel = Vec2_Add(accel, Vec2_Scale(c->tangent, c->tangent_force / b->mass));
                }
                if (c->i1 == i) {
                    accel = Vec2_Sub(accel, Vec2_Scale(c->normal, c->normal_force / b->mass));
                    accel = Vec2_Sub(accel, Vec2_Scale(c->tangent, c->tangent_force / b->mass));
                }
            }

            Vec2 p = Vec2_Add(b->center, Vec2_Scale(accel, settings.line_len));
            if (Vec2_Length(accel) > 1e-6f) DrawArrow(b->center, p);
        }
    }

    if (settings.show_axes) {
        glColor3f(0.3f, 0.3f, 1.f);
        glLineWidth(2.f);
        glBegin(GL_LINES);
        for (int i = 0; i < N_CONTACTS; i++) {
            Contact *c = &contacts[i];
            Vec2 p1 = Vec2_Add(c->pos, Vec2_Scale(c->normal, 3 * settings.line_len));
            Vec2 p2 = Vec2_Add(c->pos, Vec2_Scale(c->tangent, 3 * settings.line_len));
            DrawArrow(c->pos, p1);
            DrawArrow(c->pos, p2);
        }
        glEnd();
    }

    glColor3f(1.f, 1.f, 1.f);
    Text_Draw(10.f, 10.f,
        "W - show_ids\n"
        "1 - input mode\n"
        "2 - output mode\n"
        "3 - total mode\n"
        "Q/E - line len\n"
        "S - show axes\n",
        8.f
    );
}

void Phys_Key(int key, int action)
{
    if (action != GLFW_PRESS) return;

    switch (key) {
    case GLFW_KEY_W: settings.show_ids = !settings.show_ids; break;
    case GLFW_KEY_1: settings.mode = MODE_INPUT; break;
    case GLFW_KEY_2: settings.mode = MODE_OUTPUT; break;
    case GLFW_KEY_3: settings.mode = MODE_TOTAL; break;
    case GLFW_KEY_Q: settings.line_len /= 2.f; break;
    case GLFW_KEY_E: settings.line_len *= 2.f; break;
    case GLFW_KEY_S: settings.show_axes = !settings.show_axes; break;
    default: break; 
    }
}
