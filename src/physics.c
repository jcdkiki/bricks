#include "physics.h"

#include <float.h>
#include <glad/glad.h>
#include "text.h"

#include "linalg.h"
#include "defines.h"
#include <stdio.h>
#include <stdlib.h>
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
    int i, j;
    Vec2 normal;
    Vec2 tangent;
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

double CalcRelAccel(Matrix *A, Matrix *b, Matrix *x, int k)
{
    double res = -MATRIX_AT(*b, k, 0);
    for (int i = 0; i < x->rows; i++) {
        res += MATRIX_AT(*A, k, i) * MATRIX_AT(*x, i, 0);
    }
    return res;
}

// Ax >= b
// Helper to clamp values
double Clamp(double v, double min, double max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

static void ProjectedGaussSeidel(Matrix *A, Matrix *b, Matrix *x)
{
    int M = x->rows / 2; 

    for (int iter = 0; iter < 100; iter++) {
        
        for (int i = 0; i < x->rows; i++) {
            double new_xi = MATRIX_AT(*b, i, 0);
            
            for (int j = 0; j < x->rows; j++) {
                if (i == j) continue;
                new_xi -= MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
            }

            new_xi /= MATRIX_AT(*A, i, i);

            if (i < M) { 
                MATRIX_AT(*x, i, 0) = fmax(0.0, new_xi);
            } 
            else {
                double normal_force = MATRIX_AT(*x, i - M, 0);
                double mu = contacts[i - M].mu;
                double limit = mu * normal_force;
                MATRIX_AT(*x, i, 0) = Clamp(new_xi, -limit, limit);
            }
        }
    }
}

enum {
    AXIS_NORMAL,
    AXIS_TANGENT
};

void ProjectForces(Matrix *A, int forces, int axes)
{
    for (int k = 0; k < N_CONTACTS; k++) {
        for (int l = 0; l < N_CONTACTS; l++) {
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
    int M = N_CONTACTS;
    
    Matrix Ann, Ant, Atn, Att;
    Matrix_Init(&Ann, M, M);
    Matrix_Init(&Ant, M, M);
    Matrix_Init(&Atn, M, M);
    Matrix_Init(&Att, M, M);

    ProjectForces(&Ann, AXIS_NORMAL, AXIS_NORMAL);
    ProjectForces(&Ant, AXIS_NORMAL, AXIS_TANGENT);
    ProjectForces(&Atn, AXIS_TANGENT, AXIS_NORMAL);
    ProjectForces(&Att, AXIS_TANGENT, AXIS_TANGENT);

    Matrix A;
    Matrix_Init(&A, M * 2, M * 2);
    
    Matrix_Put(&A, &Ann, 0, 0);
    Matrix_Put(&A, &Ant, 0, M);
    Matrix_Put(&A, &Atn, M, 0);
    Matrix_Put(&A, &Att, M, M);

    Matrix b;
    Matrix_Init(&b, M * 2, 1);
    Matrix_Init(x, M * 2, 1);

    for (int k = 0; k < M; k++) {
        Contact *ck = &contacts[k];
        Vec2 nk = ck->normal;
        Vec2 tk = ck->tangent;
        Vec2 rel_accel = Vec2_Sub(bodies[ck->j].accel, bodies[ck->i].accel);

        MATRIX_AT(b, k, 0)     = -Vec2_Dot(nk, rel_accel);
        MATRIX_AT(b, k + M, 0) = -Vec2_Dot(tk, rel_accel);
    }

    ProjectedGaussSeidel(&A, &b, x);
    
    Matrix_Free(&A); Matrix_Free(&b);
    Matrix_Free(&Ann); Matrix_Free(&Ant); Matrix_Free(&Atn); Matrix_Free(&Att);
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
        .i = 0,
        .j = 1,
        .normal = {s_theta, -c_theta}, 
        .tangent = {-c_theta, -s_theta},
        .mu = 1.f,
        .pos = {500 + 25 * s_theta, 500 - 25 * c_theta}
    };

    contacts[1] = (Contact) {
        .i = 1,
        .j = 2,
        .normal = {s_theta, -c_theta},
        .tangent = {-c_theta, -s_theta},
        .mu = 0.25f,
        .pos = {500 + 75 * s_theta, 500 - 75 * c_theta}
    };

    Matrix x;
    Solve(&x);
    for (int i = 0; i < N_CONTACTS; i++) {
        contacts[i].normal_force = MATRIX_AT(x, i, 0);
        contacts[i].tangent_force = MATRIX_AT(x, i + N_CONTACTS, 0);
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
            Vec2 rel_accel = Vec2_Sub(bodies[c->j].accel, bodies[c->i].accel);
            if (rel_accel.x == 0 && rel_accel.y == 0) continue;

            Vec2 p = Vec2_Add(c->pos, Vec2_Scale(rel_accel, settings.line_len));
            DrawArrow(c->pos, p);

            double n_scale = Vec2_Dot(c->normal, rel_accel) * settings.line_len;
            double t_scale = Vec2_Dot(c->tangent, rel_accel) * settings.line_len;

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
                if (c->j == i) {
                    accel = Vec2_Add(accel, Vec2_Scale(c->normal, c->normal_force / b->mass));
                    accel = Vec2_Add(accel, Vec2_Scale(c->tangent, c->tangent_force / b->mass));
                }
                if (c->i == i) {
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
