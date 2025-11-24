#include <glad/glad.h>
#include <stdio.h>

#include "defines.h"
#include "render.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"

GLFWwindow* window;

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    Render_OnMouse(button, action, mods);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Render_OnKey(key, action, mods);
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    Render_OnScroll(xoffset, yoffset);
}

int main(void)
{
    if (!glfwInit()) {
        return -1;
    }

    float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    glfwWindowHint(GLFW_SAMPLES, 8);
    window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "bricks", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetScrollCallback(window, scrollCallback);

    if (!gladLoadGL()) {
        glfwTerminate();
        return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale); 
    style.FontScaleDpi = main_scale;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();
    
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    float last_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        float current_time = glfwGetTime();
        float dt = current_time - last_time;
        last_time = current_time;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Render_UpdateCamera(dt);
        Render_Draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
