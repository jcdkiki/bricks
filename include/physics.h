/* 
 * General formulation:
 * { mu * f_n - |f_t| >= 0          | Cone
 * { |a_t| (mu * f_n - |f_t|) = 0   | Complementarity
 * { |a_t| |f_t| = -a_t * f_t       | Direction when sliding
 *
 * LCP:
 * { a_n >= 0                 compl. f_n >= 0
 * { a_t_i + b >= 0           compl. f_t >= 0
 * { mu*f_n - sum(a_t_i) >= 0 compl. b >= 0
 *
 * Solve Ax=b
 * Constraints:
 * a_n >= 0
 * f_n >= 0
 * mu*f_n >= |a_t|
 * 
*/

#ifndef PHYSICS_H
#define PHYSICS_H

#include "linalg.h"
#include <vector>

//#define USE_LEMKE
//#define USE_PG
#define USE_ENUM

#define ENUM_RND 50000

#if defined(USE_PG)
#define N_TANGENTS 2
#else
#define N_TANGENTS 10
#endif

#if defined(USE_PG)
#define METHOD_NAME "PG"
#elif defined(USE_LEMKE)
#define METHOD_NAME "LEMKE"
#elif defined(USE_ENUM)
    #ifdef ENUM_RND
    #define METHOD_NAME "ENUM RND"
    #else
    #define METHOD_NAME "ENUM"
    #endif
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

void Phys_TangentsFromEuler();
void Phys_SaveToFile(const char *filename);
void Phys_ReadFromFile(const char *filename);
Vec3 Phys_ForceEffectOnPoint(Vec3 force_pos, Vec3 center_of_mass, Vec3 force, Vec3 point, double mass, Vec3 inertia);
Vec3 Phys_GetBodyAccel(int i, Vec3 point);

#endif
