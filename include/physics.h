#ifndef PHYSICS_H
#define PHYSICS_H

#include "linalg.h"
#include <vector>

struct Contact {
    int i, j;
    double mu;

    double angle;
    Vec2 normal;
    Vec2 tangent;

    Vec2 pos;

    double normal_force;
    double tangent_force;
};

struct Body {
    double mass;
    Vec2 accel;

    Vec2 center;
    Vec2 size;
    double angle;
};

extern std::vector<Body> bodies;
extern std::vector<Contact> contacts;

void Phys_Solve();

void Phys_SaveToFile(const char *filename);
void Phys_ReadFromFile(const char *filename);

#endif
