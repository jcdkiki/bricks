#include "physics.h"

#include <float.h>
#include <glad/glad.h>
#include "text.h"

#include "linalg.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>

#define MAX_BODIES 128
#define MAX_CONTACTS 128
#define G 98

typedef struct {
    Vec2 pos;
    Vec2 vel;
    double angle;
    double ang_vel;

    double mass;
    double ang_mass;

    int apply_gravity;
    Vec2 size;

    Vec2 force;
    double torque;
    double mu;
} Body;

typedef struct {
    Body *b1, *b2;
    Vec2 pos;
    Vec2 normal;
    Vec2 tangent;
    double depth;
    double mu;
} Contact;

Body bodies[MAX_BODIES];
Contact contacts[MAX_CONTACTS];
int n_bodies, n_contacts;

int selected_body = -1;

static void AddBrick(int i, Vec2 pos)
{
    bodies[i] = (Body) {
        .pos = pos,
        .vel = { 0, 0 },
        .angle = 0,
        .ang_vel = 0,
        .mass = 1,
        .ang_mass = (1.0f * (100*100 + 50*50)) / 12.0f,
        .apply_gravity = 1,
        .size = { 100, 50 },
        .mu = 2.f
    };
}

void Phys_Init()
{
    int si = 3, sj = 6;
    n_bodies = si*sj + 1;

    for (int i = 0; i < si; i++) {
        for (int j = 0; j < sj; j++) {
            AddBrick(i*sj + j, (Vec2) { 300 + 100*i, 550 - 50*j });
        }
    }

    bodies[si*sj] = (Body) {
        .pos = { 300, 600 }, .vel = { 0, 0 },
        .angle = 0, .ang_vel = 0,
        .mass = INFINITY, .ang_mass = INFINITY,
        .apply_gravity = 0,
        .size = { 2000, 50 },
        .mu = 2.f
    };
}

static void GetRectCorners(Body *body, Vec2 corners[4])
{
    double c = cos(body->angle);
    double s = sin(body->angle);
    double hw = body->size.x * 0.5f;
    double hh = body->size.y * 0.5f;
    
    Vec2 right = {c * hw, s * hw};
    Vec2 up = {-s * hh, c * hh};
    
    corners[0] = Vec2_Add(body->pos, Vec2_Add(right, up));
    corners[1] = Vec2_Add(body->pos, Vec2_Sub(up, right));
    corners[2] = Vec2_Sub(body->pos, Vec2_Add(right, up));
    corners[3] = Vec2_Add(body->pos, Vec2_Sub(right, up));
}

static void ProjectCornersOnAxis(Vec2 corners[4], Vec2 axis, double *min, double *max)
{
    *min = DBL_MAX;
    *max = -DBL_MAX;
    for (int i = 0; i < 4; i++) {
        double proj = Vec2_Dot(corners[i], axis);
        if (proj < *min) *min = proj;
        if (proj > *max) *max = proj;
    }
}

static int IsCornerInsideBody(Vec2 corner, Body *body, Vec2 axes[2])
{
    for (int a = 0; a < 2; a++) {
        double proj_v = Vec2_Dot(corner, axes[a]);
        double proj_c = Vec2_Dot(body->pos, axes[a]);
        double half_size = (a == 0) ? body->size.x * 0.5f : body->size.y * 0.5f;
        if (fabs(proj_v - proj_c) > half_size) return 0;
    }
    return 1;
}

static Vec2 ComputeContactPoint(Vec2 corners1[4], Vec2 corners2[4], Body *b1, Body *b2, Vec2 axes[4])
{
    Vec2 contact_sum = {0, 0};
    int contact_count = 0;
    
    for (int i = 0; i < 4; i++) {
        if (IsCornerInsideBody(corners1[i], b2, &axes[2])) {
            contact_sum = Vec2_Add(contact_sum, corners1[i]);
            contact_count++;
        }
    }
    
    for (int i = 0; i < 4; i++) {
        if (IsCornerInsideBody(corners2[i], b1, &axes[0])) {
            contact_sum = Vec2_Add(contact_sum, corners2[i]);
            contact_count++;
        }
    }
    
    return contact_count > 0 ? Vec2_Scale(contact_sum, 1.0f / contact_count)
                             : Vec2_Scale(Vec2_Add(b1->pos, b2->pos), 0.5f);
}

static int BodiesCollide(Body *b1, Body *b2, Vec2 *normal, double *depth, Vec2 *pos)
{
    Vec2 corners1[4], corners2[4];
    GetRectCorners(b1, corners1);
    GetRectCorners(b2, corners2);
    
    Vec2 axes[4] = {
        Vec2_Normalize(Vec2_Sub(corners1[0], corners1[1])),
        Vec2_Normalize(Vec2_Sub(corners1[0], corners1[3])),
        Vec2_Normalize(Vec2_Sub(corners2[0], corners2[1])),
        Vec2_Normalize(Vec2_Sub(corners2[0], corners2[3]))
    };
    
    double min_overlap = FLT_MAX;
    Vec2 min_axis = {0, 0};
    
    for (int a = 0; a < 4; a++) {
        double min1, max1, min2, max2;
        ProjectCornersOnAxis(corners1, axes[a], &min1, &max1);
        ProjectCornersOnAxis(corners2, axes[a], &min2, &max2);
        
        if (max1 - min2 < 1e-6f || max2 - min1 < 1e-6f) return 0;
        
        double overlap = fminf(max1, max2) - fmaxf(min1, min2);
        if (overlap < min_overlap) {
            min_overlap = overlap;
            min_axis = axes[a];
        }
    }
    
    *depth = min_overlap;
    *normal = Vec2_Dot(Vec2_Sub(b2->pos, b1->pos), min_axis) < 0 
              ? Vec2_Scale(min_axis, -1) : min_axis;
    *pos = ComputeContactPoint(corners1, corners2, b1, b2, axes);
    
    return 1;
}

void Phys_Draw()
{
    glDisable(GL_DEPTH_TEST);
    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        glPushMatrix();
        glTranslatef(body->pos.x, body->pos.y, 0);
        glRotatef(body->angle * 180 / M_PI, 0, 0, 1);
        
        if (i == selected_body) {
            glColor3f(1.f, 1.f, 0.f);
            glLineWidth(2.f);
        }
        else {
            glColor3f(1.f, 1.f, 1.f);
            glLineWidth(1.f);
        }
        
        glBegin(GL_LINE_LOOP);
        glVertex2f(-body->size.x / 2, -body->size.y / 2);
        glVertex2f(body->size.x / 2, -body->size.y / 2);
        glVertex2f(body->size.x / 2, body->size.y / 2);
        glVertex2f(-body->size.x / 2, body->size.y / 2);
        glEnd();
        
        glPopMatrix();
    }
    
    glColor3f(1.f, 1.f, 1.f);
    glPointSize(5.f);
    glBegin(GL_POINTS);
    glColor3f(1.f, 0.f, 0.f);
    for (int i = 0; i < n_contacts; i++) {
        glVertex2f(contacts[i].pos.x, contacts[i].pos.y);
    }
    glEnd();

    glLineWidth(2.f);
    glBegin(GL_LINES);
    for (int i = 0; i < n_contacts; i++) {
        Contact *c = &contacts[i];
        glColor3f(1.f, 0.f, 0.f);
        glVertex2f(c->pos.x + 10 * c->normal.x, c->pos.y + 10 * c->normal.y);
        glVertex2f(c->pos.x - 10 * c->normal.x, c->pos.y - 10 * c->normal.y);
        glColor3f(0.f, 1.f, 0.f);
        glVertex2f(c->pos.x + 10 * c->tangent.x, c->pos.y + 10 * c->tangent.y);
        glVertex2f(c->pos.x - 10 * c->tangent.x, c->pos.y - 10 * c->tangent.y);
    }
    glEnd();

    static char text[128];
    sprintf(text, "n_contacts: %d", n_contacts);
    Text_Draw(10.f, 10.f, text, 8.f);    
    
    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        sprintf(text, "%d: pos=(%.0f %.0f) vel=(%.0f %.0f)", i,
            body->pos.x, body->pos.y,
            body->vel.x, body->vel.y
        );
        Text_Draw(10.f, 20.f + 10.f * i, text, 8.f);
    }
}

static int PointInBody(Vec2 point, Body *body)
{
    Vec2 local = Vec2_Sub(point, body->pos);
    double c = cosf(-body->angle);
    double s = sinf(-body->angle);
    double lx = local.x * c - local.y * s;
    double ly = local.x * s + local.y * c;
    return fabs(lx) <= body->size.x * 0.5f && fabs(ly) <= body->size.y * 0.5f;
}

static void ApplyForce(Body *body, Vec2 force, Vec2 pos)
{
    Vec2 r = Vec2_Sub(pos, body->pos);
    
    double inv_mass = isinf(body->mass) ? 0.0f : 1.0f / body->mass;
    body->vel = Vec2_Add(body->vel, Vec2_Scale(force, inv_mass));
    
    double inv_ang_mass = isinf(body->ang_mass) ? 0.0f : 1.0f / body->ang_mass;
    body->ang_vel += inv_ang_mass * Vec2_Cross(r, force);
}

static void Phys_Input()
{
    double cursor_x, cursor_y;
    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    glfwGetCursorPos(window, &cursor_x, &cursor_y);
    
    Vec2 cursor = {cursor_x, cursor_y};
    
    if (state == GLFW_PRESS) {
        selected_body = -1;
        for (int i = 0; i < n_bodies; i++) {
            if (PointInBody(cursor, &bodies[i])) {
                selected_body = i;
                break;
            }
        }
    }

    if (selected_body == -1) return;
    Body *b = &bodies[selected_body];

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) ApplyForce(b, (Vec2) { -10, 0 }, b->pos);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) ApplyForce(b, (Vec2) { 10, 0 }, b->pos);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) ApplyForce(b, (Vec2) { 0, -10 }, b->pos);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) ApplyForce(b, (Vec2) { 0, 10 }, b->pos);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) b->ang_vel += 0.1f;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) b->ang_vel -= 0.1f;
}

static void ProjectedGaussSeidel(Matrix *A, Matrix *b, Matrix *x)
{
    memset(x->data, 0, sizeof(*x->data) * x->rows * x->cols);

    for (int iter = 0; iter < 10; iter++) {
        for (int i = 0; i < x->rows; i++) {
            double new_xi = MATRIX_AT(*b, i, 0);
            for (int j = 0; j < x->rows; j++) {
                if (i == j) continue;
                new_xi -= MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
            }
            MATRIX_AT(*x, i, 0) = new_xi / MATRIX_AT(*A, i, i);
        }

        for (int i = 0; i < x->rows; i++) {
            MATRIX_AT(*x, i, 0) = fmax(0, MATRIX_AT(*x, i, 0));
        }
    }
}

static void ProjectedGaussSeidelTangent(Matrix *A, Matrix *b, Matrix *x, Matrix *x_normal)
{
    memset(x->data, 0, sizeof(*x->data) * x->rows * x->cols);

    for (int iter = 0; iter < 10; iter++) {
        for (int i = 0; i < x->rows; i++) {
            double new_xi = MATRIX_AT(*b, i, 0);
            for (int j = 0; j < x->rows; j++) {
                if (i == j) continue;
                new_xi -= MATRIX_AT(*A, i, j) * MATRIX_AT(*x, j, 0);
            }
            MATRIX_AT(*x, i, 0) = new_xi / MATRIX_AT(*A, i, i);
        }

        for (int i = 0; i < x->rows; i++) {
            double lim = contacts[i].mu * MATRIX_AT(*x_normal, i, 0);
            MATRIX_AT(*x, i, 0) = fmin(lim, fmax(-lim, MATRIX_AT(*x, i, 0)));
        }
    }
}

static void InitInvMassMatrix(Matrix *M)
{
    Matrix_Init(M, n_bodies * 3, n_bodies * 3);
    for (int i = 0; i < n_bodies; i++) {
        Body *b = &bodies[i];
        double inv_mass     = isinf(b->mass)     ? 0.0f : 1.0f / b->mass;
        double inv_ang_mass = isinf(b->ang_mass) ? 0.0f : 1.0f / b->ang_mass;

        MATRIX_AT(*M, i*3 + 0, i*3 + 0) = inv_mass;
        MATRIX_AT(*M, i*3 + 1, i*3 + 1) = inv_mass;
        MATRIX_AT(*M, i*3 + 2, i*3 + 2) = inv_ang_mass;
    }
}

enum {
    AXIS_NORMAL,
    AXIS_TANGENT
};

static void InitJacobianMatrix(Matrix *J, int axis)
{
    Matrix_Init(J, n_contacts, n_bodies * 3);
    memset(J->data, 0, sizeof(*J->data) * J->rows * J->cols);

    for (int i = 0; i < n_contacts; i++) {
        Contact *c = &contacts[i];
        Vec2 r1 = Vec2_Sub(c->pos, c->b1->pos);
        Vec2 r2 = Vec2_Sub(c->pos, c->b2->pos);
        int i1 = c->b1 - bodies;
        int i2 = c->b2 - bodies;

        Vec2 axis_vec = (axis == AXIS_NORMAL) ? c->normal : c->tangent;

        MATRIX_AT(*J, i, i1*3 + 0) = axis_vec.x;
        MATRIX_AT(*J, i, i1*3 + 1) = axis_vec.y;
        MATRIX_AT(*J, i, i1*3 + 2) = Vec2_Cross(r1, axis_vec);
        MATRIX_AT(*J, i, i2*3 + 0) = -axis_vec.x;
        MATRIX_AT(*J, i, i2*3 + 1) = -axis_vec.y;
        MATRIX_AT(*J, i, i2*3 + 2) = -Vec2_Cross(r2, axis_vec);
    }
}

static Vec2 GetRelativeVelocity(Contact *c)
{
    Vec2 r1 = Vec2_Sub(c->pos, c->b1->pos);
    Vec2 r2 = Vec2_Sub(c->pos, c->b2->pos);
    
    Vec2 v1 = Vec2_Add(c->b1->vel, (Vec2){-c->b1->ang_vel * r1.y, c->b1->ang_vel * r1.x});
    Vec2 v2 = Vec2_Add(c->b2->vel, (Vec2){-c->b2->ang_vel * r2.y, c->b2->ang_vel * r2.x});
    return Vec2_Sub(v1, v2);
}

static void InitBVectorNormal(Matrix *b)
{
    Matrix_Init(b, n_contacts, 1);
    for (int i = 0; i < n_contacts; i++) {
        Contact *c = &contacts[i];
        Vec2 v = GetRelativeVelocity(c);

        double bias = (c->depth > 0.01f) ? (0.2f * (c->depth - 0.01f) / DT) : 0.f;
        MATRIX_AT(*b, i, 0) = Vec2_Dot(v, c->normal) + bias;
    }
}

static void InitBVectorTangent(Matrix *b)
{
    Matrix_Init(b, n_contacts, 1);
    
    for (int i = 0; i < n_contacts; i++) {
        Contact *c = &contacts[i];
        Vec2 v = GetRelativeVelocity(c);
        MATRIX_AT(*b, i, 0) = -Vec2_Dot(v, c->tangent);
    }
}

static void SolveNormalImpulse(Matrix *x)
{
    Matrix M, J, JT, JM;
    Matrix A, b;

    InitInvMassMatrix(&M);
    InitJacobianMatrix(&J, AXIS_NORMAL);
    Matrix_InitTransposed(&J, &JT);
    Matrix_Init(&JM, n_contacts, n_bodies * 3);
    
    Matrix_Init(&A, n_contacts, n_contacts);
    InitBVectorNormal(&b);
    Matrix_Init(x, n_contacts, 1);
    
    Matrix_Mul(&J, &M, &JM);
    Matrix_Mul(&JM, &JT, &A);

    ProjectedGaussSeidel(&A, &b, x);

    Matrix_Free(&M); Matrix_Free(&J); Matrix_Free(&JT); Matrix_Free(&JM);
    Matrix_Free(&A); Matrix_Free(&b);
}

static void SolveTangentImpulse(Matrix *x, Matrix *x_normal)
{
    Matrix M, J, JT, JM;
    Matrix A, b;

    InitInvMassMatrix(&M);
    InitJacobianMatrix(&J, AXIS_TANGENT);
    Matrix_InitTransposed(&J, &JT);
    Matrix_Init(&JM, n_contacts, n_bodies * 3);
    
    Matrix_Init(&A, n_contacts, n_contacts);
    InitBVectorTangent(&b);
    Matrix_Init(x, n_contacts, 1);

    Matrix_Mul(&J, &M, &JM);
    Matrix_Mul(&JM, &JT, &A);
    
    ProjectedGaussSeidelTangent(&A, &b, x, x_normal);

    Matrix_Free(&M); Matrix_Free(&J); Matrix_Free(&JT); Matrix_Free(&JM);
    Matrix_Free(&A); Matrix_Free(&b);
}

static void ResolveContacts()
{
    if (n_contacts == 0) return;

    Matrix x_normal, x_tangent;
    SolveNormalImpulse(&x_normal);
    SolveTangentImpulse(&x_tangent, &x_normal);
    
    for (int i = 0; i < n_contacts; i++) {
        Contact *c = &contacts[i];
        double normal_impulse = MATRIX_AT(x_normal, i, 0);
        double tangent_impulse = MATRIX_AT(x_tangent, i, 0);

        Vec2 impulse = Vec2_Add(
            Vec2_Scale(c->normal, normal_impulse),
            Vec2_Scale(c->tangent, -tangent_impulse)
        );

        ApplyForce(c->b1, Vec2_Scale(impulse, -1), c->pos);
        ApplyForce(c->b2, impulse, c->pos);
    }

    Matrix_Free(&x_normal); Matrix_Free(&x_tangent);
}

void Phys_Tick()
{
    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        body->torque = 0;
        body->force = (Vec2){0, 0};

        Vec2 gravity = {0, G * body->mass};
        if (body->apply_gravity) ApplyForce(body, Vec2_Scale(gravity, DT), body->pos);
    }

    Phys_Input();

    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        body->ang_vel += body->torque / body->ang_mass;
        body->vel = Vec2_Add(body->vel, Vec2_Scale(body->force, 1.f / body->mass));
    }

    n_contacts = 0;
    for (int i = 0; i < n_bodies; i++) {
        for (int j = i + 1; j < n_bodies; j++) {
            Vec2 normal, pos;
            double depth;

            if (BodiesCollide(&bodies[i], &bodies[j], &normal, &depth, &pos)) {
                contacts[n_contacts] = (Contact) {
                    .b1 = &bodies[i], .b2 = &bodies[j], .pos = pos,
                    .normal = normal,
                    .tangent = (Vec2) { -normal.y, normal.x },
                    .depth = depth,
                    .mu = sqrtf(bodies[i].mu * bodies[j].mu)
                };
                n_contacts++;
            }
        }
    }

    ResolveContacts();

    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        body->angle += body->ang_vel * DT;
        body->pos = Vec2_Add(body->pos, Vec2_Scale(body->vel, DT));
    }
}
