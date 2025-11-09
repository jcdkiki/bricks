#include <glad/glad.h>
#include <stdio.h>

#include "text.h"
#include "defines.h"
#include "physics.h"

GLFWwindow* window;

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Phys_Key(key, action);
}

int main(void)
{
    if (!glfwInit()) {
        return -1;
    }

    window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "bricks", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, KeyCallback);

    if (!gladLoadGL()) {
        glfwTerminate();
        return -1;
    }

    Text_Init();
    Phys_Init();
    
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    float tick_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, WIN_WIDTH, WIN_HEIGHT, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        Phys_Draw();

        glEnable(GL_DEPTH_TEST);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
