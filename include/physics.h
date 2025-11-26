#ifndef PHYSICS_H
#define PHYSICS_H

#include "linalg.h"
#include <vector>

#define N_TANGENTS 9

#define AXIS_NORMAL 0
#define AXIS_TANGENT1 1
#define AXIS_TANGENT2 2

struct Contact {
    int i, j;
    double mu;

    Vec3 axes[N_TANGENTS + 1];

    Vec3 pos;
    Vec3 angles;

    double res_normal_force;
    double res_tangent_force[N_TANGENTS];
};

struct Body {
    double mass;
    Vec3 accel;
    
    Vec3 center;
    Vec3 size;
    Vec3 euler;
};

extern std::vector<Body> bodies;
extern std::vector<Contact> contacts;

void Phys_Solve();

void Phys_SaveToFile(const char *filename);
void Phys_ReadFromFile(const char *filename);

#endif
