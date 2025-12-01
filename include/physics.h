#ifndef PHYSICS_H
#define PHYSICS_H

#include "linalg.h"
#include <vector>

//#define USE_LEMKE
//#define USE_PGS
#define USE_PG

#if defined(USE_PGS) || defined(USE_PG)
#define N_TANGENTS 2
#else
#define N_TANGENTS 3
#endif

#if defined(USE_PG)
#define METHOD_NAME "PG"
#elif defined(USE_PGS)
#define METHOD_NAME "PGS"
#elif defined(USE_LEMKE)
#define METHOD_NAME "LEMKE"
#endif

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
    Vec3 force;
    
    Vec3 center;
    Vec3 size;
    Vec3 euler;
};

extern std::vector<Body> bodies;
extern std::vector<Contact> contacts;

void Phys_Solve();

void Phys_SaveToFile(const char *filename);
void Phys_ReadFromFile(const char *filename);
Vec3 Phys_ForceEffectOnPoint(Vec3 force_pos, Vec3 center_of_mass, Vec3 force, Vec3 point, double mass, Vec3 inertia);

#endif
