#ifndef DEFINES_H
#define DEFINES_H

#include "GLFW/glfw3.h"

#define WIN_WIDTH 1024
#define WIN_HEIGHT 768

#define ASPECT ((float)(WIN_WIDTH) / (float)(WIN_HEIGHT))
#define FOV 60.f
#define FAR 100.f
#define NEAR 0.1f
#define DT (1.f / 60.f)

extern GLFWwindow* window;

#endif
