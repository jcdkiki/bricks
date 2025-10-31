#include <glad/glad.h>
#include <stdio.h>

#include "text.h"
#include "defines.h"
#include "physics.h"

GLFWwindow* window;
int is_paused = 0;

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_E && action == GLFW_PRESS || action == GLFW_REPEAT)
        Phys_Tick();
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
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
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

        if (!is_paused) {
            float cur_time = glfwGetTime();
            while (cur_time - tick_time > DT) {
                Phys_Tick();
                cur_time = glfwGetTime();
                tick_time += DT;
            }
        }
    }

    glfwTerminate();
    return 0;
}
