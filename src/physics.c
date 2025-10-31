#include "physics.h"

#include <float.h>
#include <glad/glad.h>
#include "text.h"

#include "linalg.h"
#include "defines.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_BODIES 16
#define MAX_CONTACTS 16
#define G 980
#define MU 1.0

typedef struct {
    Vec2f pos;
    Vec2f vel;
    float angle;
    float ang_vel;

    float mass;
    float ang_mass;

    int apply_gravity;
    Vec2f size;

    Vec2f force;
    float torque;
} Body;

typedef enum {
    FRICTION_STATIC,
    FRICTION_DYNAMIC,
} FrictionType;

typedef struct {
    Body *b1, *b2;
    Vec2f pos;
    Vec2f normal;
    Vec2f tangent;
    FrictionType friction_type;
} Contact;

Body bodies[MAX_BODIES];
Contact contacts[MAX_CONTACTS];
int n_bodies, n_contacts;

int grabbed_body = -1;
Vec2f grab_offset;
double cursor_x, cursor_y;

void Phys_Init()
{
    n_bodies = 4;

    bodies[0] = (Body) {
        .pos = { 300, 300 },
        .vel = { 0, 0 },
        .angle = 0,
        .ang_vel = 0,
        .mass = 1,
        .ang_mass = (1.0f * (100*100 + 50*50)) / 12.0f,
        .apply_gravity = 1,
        .size = { 100, 50 }
    };

    bodies[1] = (Body) {
        .pos = { 300, 400 },
        .vel = { 0, 0 },
        .angle = 0,
        .ang_vel = 0,
        .mass = 1,
        .ang_mass = (1.0f * (100*100 + 50*50)) / 12.0f,
        .apply_gravity = 1,
        .size = { 100, 50 }
    };

    bodies[2] = (Body) {
        .pos = { 500, 600 },
        .vel = { 0, 0 },
        .angle = 0, // M_PI / 6.f,
        .ang_vel = 0,
        .mass = INFINITY,
        .ang_mass = INFINITY,
        .apply_gravity = 0,
        .size = { 2000, 50 }
    };

    bodies[3] = (Body) {
        .pos = { 800, 700 },
        .vel = { 0, 0 },
        .angle = 0,
        .ang_vel = 0,
        .mass = INFINITY,
        .ang_mass = INFINITY,
        .apply_gravity = 0,
        .size = { 50, 2000 }
    };
}

static void GetRectCorners(Body *body, Vec2f corners[4])
{
    float c = cosf(body->angle);
    float s = sinf(body->angle);
    float hw = body->size.x * 0.5f;
    float hh = body->size.y * 0.5f;
    
    Vec2f right = {c * hw, s * hw};
    Vec2f up = {-s * hh, c * hh};
    
    corners[0] = Vec2_Add(body->pos, Vec2_Add(right, up));
    corners[1] = Vec2_Add(body->pos, Vec2_Sub(up, right));
    corners[2] = Vec2_Sub(body->pos, Vec2_Add(right, up));
    corners[3] = Vec2_Add(body->pos, Vec2_Sub(right, up));
}

static void ProjectCornersOnAxis(Vec2f corners[4], Vec2f axis, float *min, float *max)
{
    *min = FLT_MAX;
    *max = -FLT_MAX;
    for (int i = 0; i < 4; i++) {
        float proj = Vec2_Dot(corners[i], axis);
        if (proj < *min) *min = proj;
        if (proj > *max) *max = proj;
    }
}

static int IsCornerInsideBody(Vec2f corner, Body *body, Vec2f axes[2])
{
    for (int a = 0; a < 2; a++) {
        float proj_v = Vec2_Dot(corner, axes[a]);
        float proj_c = Vec2_Dot(body->pos, axes[a]);
        float half_size = (a == 0) ? body->size.x * 0.5f : body->size.y * 0.5f;
        if (fabsf(proj_v - proj_c) > half_size) return 0;
    }
    return 1;
}

static Vec2f ComputeContactPoint(Vec2f corners1[4], Vec2f corners2[4], Body *b1, Body *b2, Vec2f axes[4])
{
    Vec2f contact_sum = {0, 0};
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

static int BodiesCollide(Body *b1, Body *b2, Vec2f *normal, float *depth, Vec2f *pos)
{
    Vec2f corners1[4], corners2[4];
    GetRectCorners(b1, corners1);
    GetRectCorners(b2, corners2);
    
    Vec2f axes[4] = {
        Vec2_Normalize(Vec2_Sub(corners1[0], corners1[1])),
        Vec2_Normalize(Vec2_Sub(corners1[0], corners1[3])),
        Vec2_Normalize(Vec2_Sub(corners2[0], corners2[1])),
        Vec2_Normalize(Vec2_Sub(corners2[0], corners2[3]))
    };
    
    float min_overlap = FLT_MAX;
    Vec2f min_axis = {0, 0};
    
    for (int a = 0; a < 4; a++) {
        float min1, max1, min2, max2;
        ProjectCornersOnAxis(corners1, axes[a], &min1, &max1);
        ProjectCornersOnAxis(corners2, axes[a], &min2, &max2);
        
        if (max1 < min2 || max2 < min1) return 0;
        
        float overlap = fminf(max1, max2) - fmaxf(min1, min2);
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
    glLineWidth(1.f);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1.f, 1.f, 1.f);
    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        glPushMatrix();
        glTranslatef(body->pos.x, body->pos.y, 0);
        glRotatef(body->angle * 180 / M_PI, 0, 0, 1);

        glBegin(GL_LINE_LOOP);
        glVertex2f(-body->size.x / 2, -body->size.y / 2);
        glVertex2f(body->size.x / 2, -body->size.y / 2);
        glVertex2f(body->size.x / 2, body->size.y / 2);
        glVertex2f(-body->size.x / 2, body->size.y / 2);
        glEnd();

        glPopMatrix();
    }

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

    if (grabbed_body != -1) {
        Body *body = &bodies[grabbed_body];
        glColor3f(1.f, 1.f, 0.f);
        glBegin(GL_LINES);
        glVertex2f(body->pos.x + grab_offset.x, body->pos.y + grab_offset.y);
        glVertex2f(cursor_x, cursor_y);
        glEnd();
    }

    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        static char text[128];
        sprintf(text, "%d: pos=(%.0f %.0f) vel=(%.0f %.0f)", i,
            body->pos.x, body->pos.y,
            body->vel.x, body->vel.y
        );
        Text_Draw(10.f, 10.f + 10.f * i, text, 8.f);
    }
}

static int PointInBody(Vec2f point, Body *body)
{
    Vec2f local = Vec2_Sub(point, body->pos);
    float c = cosf(-body->angle);
    float s = sinf(-body->angle);
    float lx = local.x * c - local.y * s;
    float ly = local.x * s + local.y * c;
    return fabsf(lx) <= body->size.x * 0.5f && fabsf(ly) <= body->size.y * 0.5f;
}

static void ApplyForce(Body *body, Vec2f force, Vec2f pos)
{
    Vec2f r = Vec2_Sub(pos, body->pos);
    body->force = Vec2_Add(body->force, force);
    body->torque += r.x * force.y - r.y * force.x;
}

static void Phys_Input()
{
    static int prev_state;
    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    glfwGetCursorPos(window, &cursor_x, &cursor_y);
    
    Vec2f cursor = {cursor_x, cursor_y};
    
    if (state == GLFW_PRESS && prev_state == GLFW_RELEASE) {
        for (int i = 0; i < n_bodies; i++) {
            if (PointInBody(cursor, &bodies[i])) {
                grabbed_body = i;
                grab_offset = Vec2_Sub(cursor, bodies[i].pos);
                break;
            }
        }
    }
    else if (state == GLFW_RELEASE && prev_state == GLFW_PRESS && grabbed_body != -1) {
        Body *body = &bodies[grabbed_body];
        Vec2f target = Vec2_Sub(cursor, grab_offset);
        ApplyForce(body, Vec2_Sub(target, body->pos), Vec2_Add(body->pos, grab_offset));
        grabbed_body = -1;
    }

    prev_state = state;
}

static void ResolveCollision(Body *b1, Body *b2, Vec2f normal, float depth, Vec2f pos)
{
    Vec2f r1 = Vec2_Sub(pos, b1->pos);
    Vec2f r2 = Vec2_Sub(pos, b2->pos);
    
    Vec2f vel1 = Vec2_Add(b1->vel, (Vec2f){-b1->ang_vel * r1.y, b1->ang_vel * r1.x});
    Vec2f vel2 = Vec2_Add(b2->vel, (Vec2f){-b2->ang_vel * r2.y, b2->ang_vel * r2.x});
    
    float v1_along_normal = Vec2_Dot(vel1, normal);
    float v2_along_normal = Vec2_Dot(vel2, normal);
    float total_vel = fabsf(v1_along_normal) + fabsf(v2_along_normal);
    
    if (total_vel == 0) return;
    
    float ratio1 = fabsf(v1_along_normal) / total_vel;
    float ratio2 = fabsf(v2_along_normal) / total_vel;
    
    b1->pos = Vec2_Sub(b1->pos, Vec2_Scale(normal, depth * ratio1));
    b2->pos = Vec2_Add(b2->pos, Vec2_Scale(normal, depth * ratio2));
    
    float inv_mass1 = 1.0f / b1->mass;
    float inv_mass2 = 1.0f / b2->mass;
    float inv_mass_sum = inv_mass1 + inv_mass2;
    
    float inv_ang_mass1 = isinf(b1->ang_mass) ? 0 : 1.0f / b1->ang_mass;
    float inv_ang_mass2 = isinf(b2->ang_mass) ? 0 : 1.0f / b2->ang_mass;
    
    float vel_along_normal = Vec2_Dot(Vec2_Sub(vel1, vel2), normal);
    
    float r1_cross_n = r1.x * normal.y - r1.y * normal.x;
    float r2_cross_n = r2.x * normal.y - r2.y * normal.x;
    float denom = inv_mass_sum + r1_cross_n * r1_cross_n * inv_ang_mass1 
                  + r2_cross_n * r2_cross_n * inv_ang_mass2;
    float impulse = -vel_along_normal / denom;
    
    Vec2f impulse_vec = Vec2_Scale(normal, impulse);
    ApplyForce(b1, impulse_vec, pos);
    ApplyForce(b2, Vec2_Scale(impulse_vec, -1), pos);

    Vec2f tangent = {-normal.y, normal.x};
    float vel_along_tangent = Vec2_Dot(Vec2_Sub(vel1, vel2), tangent);
    
    if (fabsf(vel_along_tangent) < 1) {
        contacts[n_contacts++] = (Contact) {
            .b1 = b1, .b2 = b2, .pos = pos,
            .normal = normal,
            .tangent = tangent
        };
    }
    else {
        float max_friction = MU * fabsf(impulse);
        float j_f = -(vel_along_tangent > 0 ? 1.0f : -1.0f) * max_friction;
        Vec2f friction_impulse = Vec2_Scale(tangent, j_f);
        ApplyForce(b1, friction_impulse, pos);
        ApplyForce(b2, Vec2_Scale(friction_impulse, -1), pos);
    }
}

void InitInvMassMatrix(Matrix *M_inv)
{
    Matrix_Init(M_inv, 3 * n_bodies, 3 * n_bodies);
    memset(M_inv->data, 0, sizeof(float) * M_inv->rows * M_inv->cols);

    for (int i = 0; i < n_bodies; i++) {
        int idx = 3 * i;
        MATRIX_AT(*M_inv, idx, idx) = 1.f / bodies[i].mass;
        MATRIX_AT(*M_inv, idx + 1, idx + 1) = 1.f / bodies[i].mass;
        MATRIX_AT(*M_inv, idx + 2, idx + 2) = 1.f / bodies[i].ang_mass;
    }
}

void InitDMatrix(Matrix *D)
{
    Matrix_Init(D, n_contacts, 3 * n_bodies);
    memset(D->data, 0, sizeof(float) * D->rows * D->cols);

    for (int k = 0; k < n_contacts; k++) {
        Contact *c = &contacts[k];
        Vec2f r1 = {c->pos.x - c->b1->pos.x, c->pos.y - c->b1->pos.y};
        Vec2f r2 = {c->pos.x - c->b2->pos.x, c->pos.y - c->b2->pos.y};
        Vec2f t = c->tangent;

        float cross1 = r1.x * t.y - r1.y * t.x;
        float cross2 = r2.x * t.y - r2.y * t.x;

        int i1 = 3 * (c->b1 - bodies);
        int i2 = 3 * (c->b2 - bodies);

        MATRIX_AT(*D, k, i1) = -t.x;
        MATRIX_AT(*D, k, i1 + 1) = -t.y;
        MATRIX_AT(*D, k, i1 + 2) = -cross1;

        MATRIX_AT(*D, k, i2) = t.x;
        MATRIX_AT(*D, k, i2 + 1) = t.y;
        MATRIX_AT(*D, k, i2 + 2) = cross2;
    }
}

void InitWMatrix(Matrix *D, Matrix *M_inv, Matrix *W)
{
    Matrix D_T;
    Matrix_InitTransposed(D, &D_T);

    Matrix temp;
    Matrix_Init(&temp, M_inv->rows, D_T.cols);
    Matrix_Mul(M_inv, &D_T, &temp);

    Matrix_Init(W, D->rows, temp.cols);
    Matrix_Mul(D, &temp, W);

    Matrix_Free(&D_T);
    Matrix_Free(&temp);
}

void Phys_Friction()
{
    Matrix M, D, W;
    InitInvMassMatrix(&M);
    InitDMatrix(&D);
    InitWMatrix(&D, &M, &W);

    Matrix_Free(&M);
    Matrix_Free(&D);
    Matrix_Free(&W);
}

void Phys_Tick()
{
    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        body->torque = 0;
        body->force = (Vec2f){0, 0};
        if (body->apply_gravity) ApplyForce(body, (Vec2f){0, G * body->mass * DT}, body->pos);
    }

    Phys_Input();

    n_contacts = 0;
    for (int i = 0; i < n_bodies; i++) {
        for (int j = i + 1; j < n_bodies; j++) {
            Vec2f normal, pos;
            Vec2f tangent;
            float depth;

            if (BodiesCollide(&bodies[i], &bodies[j], &normal, &depth, &pos)) {
                ResolveCollision(&bodies[i], &bodies[j], normal, depth, pos);
            }
        }
    }

    Phys_Friction();

    for (int i = 0; i < n_bodies; i++) {
        Body *body = &bodies[i];
        body->angle += body->ang_vel * DT;
        body->pos = Vec2_Add(body->pos, Vec2_Scale(body->vel, DT));
        body->vel = Vec2_Add(body->vel, Vec2_Scale(body->force, 1.f / body->mass * DT));
        body->ang_vel += body->torque / body->ang_mass * DT;
    }
}
