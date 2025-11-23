#include "render.h"

#include <glad/glad.h>
#include <cstdio>
#include <cmath>
#include <GLFW/glfw3.h>
#include <string>

#include "linalg.h"
#include "physics.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"
#include "imgui.h"

struct Settings {
    float line_len { 5.f };
    bool show_ids {true};
    bool show_axes { true };
    bool show_input_forces { false };
    bool show_output_forces { false };
    bool show_total_forces { false };
    bool show_rel_accel { true };

    char filename[128] { "examples/1.bin" };
    int selected_body { -1 };
    int selected_contact { -1 };
};

static Settings settings;

static void DrawArrow(Vec2 p1, Vec2 p2)
{
    glLineWidth(3.f);
    glBegin(GL_LINES);
    glVertex2f(p1.x, p1.y);
    glVertex2f(p2.x, p2.y);
    
    Vec2 dir = Vec2_Normalize(Vec2_Sub(p2, p1));
    Vec2 perp = {-dir.y, dir.x};
    Vec2 tip1 = Vec2_Add(p2, Vec2_Scale(Vec2_Add(Vec2_Scale(dir, -1), perp), 5.f));
    Vec2 tip2 = Vec2_Add(p2, Vec2_Scale(Vec2_Sub(Vec2_Scale(dir, -1), perp), 5.f));
    
    glVertex2f(p2.x, p2.y);
    glVertex2f(tip1.x, tip1.y);
    glVertex2f(p2.x, p2.y);
    glVertex2f(tip2.x, tip2.y);
    glEnd();
}

Vec2 GetBodyAccel(int i)
{
    Vec2 accel = bodies[i].accel;
    double mass = bodies[i].mass;

    for (int j = 0; j < contacts.size(); j++) {
        Contact &c = contacts[j];
        if (c.j == i) {
            accel = Vec2_Add(accel, Vec2_Scale(c.normal, c.normal_force / mass));
            accel = Vec2_Add(accel, Vec2_Scale(c.tangent, c.tangent_force / mass));
        }
        else if (c.i == i) {
            accel = Vec2_Sub(accel, Vec2_Scale(c.normal, c.normal_force / mass));
            accel = Vec2_Sub(accel, Vec2_Scale(c.tangent, c.tangent_force / mass));
        }
    }
    return accel;
}

void Render_Draw()
{
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_MULTISAMPLE);

    glDisable(GL_DEPTH_TEST);
    
    for (int i = 0; i < bodies.size(); i++) {
        if (i == settings.selected_body) {
            glColor3f(1.f, 1.f, 0.f);
            glLineWidth(2.f);
        }
        else {
            glColor3f(1.f, 1.f, 1.f);
            glLineWidth(1.f);
        }
        
        glPushMatrix();
        glTranslated(bodies[i].center.x, bodies[i].center.y, 0);
        glRotated(bodies[i].angle * 180.f / M_PI, 0, 0, 1);
        glBegin(GL_LINE_LOOP);
        glVertex2d(bodies[i].size.x / 2, bodies[i].size.y / 2);
        glVertex2d(bodies[i].size.x / 2, -bodies[i].size.y / 2);
        glVertex2d(-bodies[i].size.x / 2, -bodies[i].size.y / 2);
        glVertex2d(-bodies[i].size.x / 2, bodies[i].size.y / 2);
        glEnd();
        glPopMatrix();
    }

    glLineWidth(1.f);
    if (settings.show_input_forces) {
        glColor3f(1.f, 0.3f, 0.3f);
        for (int i = 0; i < bodies.size(); i++) {
            Body *b = &bodies[i];
            if (b->accel.x == 0 && b->accel.y == 0) continue;

            Vec2 p = Vec2_Add(b->center, Vec2_Scale(b->accel, settings.line_len));
            DrawArrow(b->center, p);
        }
    
        glColor3f(0.3f, 1.f, 0.3f);
        for (int i = 0; i < contacts.size(); i++) {
            Contact *c = &contacts[i];
            Vec2 rel_accel = Vec2_Sub(bodies[c->j].accel, bodies[c->i].accel);
            if (rel_accel.x == 0 && rel_accel.y == 0) continue;

            Vec2 p = Vec2_Add(c->pos, Vec2_Scale(rel_accel, settings.line_len));
            DrawArrow(c->pos, p);
        }
    }

    if (settings.show_output_forces) {
        glColor3f(1.f, 1.f, 0.3f);
        for (int i = 0; i < contacts.size(); i++) {
            Contact *c = &contacts[i];
            Vec2 p1 = Vec2_Add(c->pos, Vec2_Scale(c->normal, c->normal_force * settings.line_len));
            Vec2 p2 = Vec2_Add(c->pos, Vec2_Scale(c->tangent, c->tangent_force * settings.line_len));
            if (fabs(c->normal_force) > 1e-4) DrawArrow(c->pos, p1);
            if (fabs(c->tangent_force) > 1e-4) DrawArrow(c->pos, p2);
        }
    }

    if (settings.show_total_forces) {
        glColor3f(1.f, 1.f, 1.f);
        for (int i = 0; i < bodies.size(); i++) {
            Body *b = &bodies[i];
            Vec2 accel = GetBodyAccel(i);
            Vec2 p = Vec2_Add(b->center, Vec2_Scale(accel, settings.line_len));
            if (Vec2_Length(accel) > 1e-6f) DrawArrow(b->center, p);
        }
    }

    if (settings.show_rel_accel) {
        glColor3f(1.f, 1.f, 1.f);
        for (int i = 0; i < contacts.size(); i++) {
            Contact &c = contacts[i];
            Vec2 i_accel = GetBodyAccel(c.i);
            Vec2 j_accel = GetBodyAccel(c.j);
            Vec2 diff = Vec2_Scale(Vec2_Sub(j_accel, i_accel), settings.line_len);

            if (Vec2_Length(diff) > 1e-6f)
                DrawArrow(c.pos, Vec2_Add(c.pos, diff));
        }
    }

    if (settings.show_axes) {
        for (int i = 0; i < contacts.size(); i++) {
            if (settings.selected_contact == i) {
                glColor3f(1.f, 1.f, 0.f);
                glLineWidth(4.f);
            }
            else {
                glColor3f(0.3f, 0.3f, 1.f);
                glLineWidth(2.f);
            }
            
            glBegin(GL_LINES);
            Contact *c = &contacts[i];
            Vec2 p1 = Vec2_Add(c->pos, Vec2_Scale(c->normal, 15));
            Vec2 p2 = Vec2_Add(c->pos, Vec2_Scale(c->tangent, 15));
            glVertex2f(c->pos.x, c->pos.y); glVertex2f(p1.x, p1.y);
            glVertex2f(c->pos.x, c->pos.y); glVertex2f(p2.x, p2.y);
            glEnd();
        }
    }

    glColor3f(1.f, 1.f, 1.f);
    
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Settings", NULL, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Checkbox("Show IDs", &settings.show_ids);
    ImGui::Checkbox("Show Axes", &settings.show_axes);
    ImGui::Checkbox("Show input forces", &settings.show_input_forces);
    ImGui::Checkbox("Show output forces", &settings.show_output_forces);
    ImGui::Checkbox("Show total forces", &settings.show_total_forces);
    ImGui::Checkbox("Show relative acceleration", &settings.show_rel_accel);
    ImGui::InputText("Filename", settings.filename, sizeof(settings.filename));
    
    if (ImGui::Button("Load")) {
        Phys_ReadFromFile(settings.filename);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        Phys_SaveToFile(settings.filename);
    }
    ImGui::SameLine();
    if (ImGui::Button("New")) {
        contacts.clear();
        bodies.clear();
    }
    if (ImGui::Button("Solve")) {
        Phys_Solve();
    }

    ImGui::InputFloat("Line length", &settings.line_len, 0.1f, 1.f, "%.3f");

    ImGui::Separator();
    static float val;
    ImGui::InputFloat("Value", &val);

    static char buf[128];
    sprintf(buf, "cos(deg): %f\nsin(deg): %f", cos(val / 180 * M_PI), sin(val / 180 * M_PI));
    ImGui::InputTextMultiline("calc", buf, 128, ImVec2(0, 40), ImGuiInputTextFlags_ReadOnly);
    
    ImGui::Separator();
    if (ImGui::Button("New contact point")) {
        contacts.push_back(Contact {
            0, 0, 1.0, 0.0, {0.0, -1.0}, {-1.0, 0.0}, {500.0, 500.0}, 0.0, 0.0
        });
    }
    if (ImGui::Button("New body")) {
        bodies.push_back(Body {
            1.0, {0.0, 0.0}, {500.0, 500.0}, {100.0, 50.0}, 0.0 
        });
    }
    
    ImGui::End();

    if (settings.selected_body != -1) {
        ImGui::Begin("Body", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Body %d", settings.selected_body);
        Body *b = &bodies[settings.selected_body];
        ImGui::InputDouble("Mass", &b->mass); ImGui::SameLine();
        if (ImGui::Button("INF")) {
            b->mass = INFINITY;
        }

        ImGui::InputDouble("Angle", &b->angle);
        ImGui::SameLine();
        if (ImGui::Button("To deg")) {
            b->angle *= 180 / M_PI;
        }
        ImGui::SameLine();
        if (ImGui::Button("To rad")) {
            b->angle *= M_PI / 180;
        }

        ImGui::Text("pos");   ImGui::SameLine(); ImGui::InputDouble("px", &b->center.x);
                              ImGui::SameLine(); ImGui::InputDouble("py", &b->center.y);
        ImGui::Text("accel"); ImGui::SameLine(); ImGui::InputDouble("ax", &b->accel.x);
                              ImGui::SameLine(); ImGui::InputDouble("ay", &b->accel.y);
        ImGui::SameLine();
        if (ImGui::Button("G")) {
            b->accel = {0.0, 9.8};
        }

        ImGui::Text("size");  ImGui::SameLine(); ImGui::InputDouble("sx", &b->size.x);
                              ImGui::SameLine(); ImGui::InputDouble("sy", &b->size.y);

        if (ImGui::Button("Delete")) {
            bodies.erase(bodies.begin() + settings.selected_body);
            
            for (auto it = contacts.begin(); it != contacts.end();) {
                if (it->i == settings.selected_body || it->j == settings.selected_body) {
                    it = contacts.erase(it);
                }
                else {
                    if (it->i > settings.selected_body) it->i--;
                    if (it->j > settings.selected_body) it->j--;
                    it++;
                }
            }

            settings.selected_body = -1;
        }
            
        ImGui::End();
    }

    if (settings.selected_contact != -1) {
        ImGui::Begin("Contact", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Contact %d", settings.selected_contact);
        Contact *c = &contacts[settings.selected_contact];

        ImGui::InputInt("Body i", &c->i);
        ImGui::InputInt("Body j", &c->j);
        ImGui::InputDouble("Mu", &c->mu);

        ImGui::Text("pos");     ImGui::SameLine(); ImGui::InputDouble("px", &c->pos.x);
                                ImGui::SameLine(); ImGui::InputDouble("py", &c->pos.y);
        ImGui::Text("normal");  ImGui::SameLine(); ImGui::InputDouble("nx", &c->normal.x);
                                ImGui::SameLine(); ImGui::InputDouble("ny", &c->normal.y);
        ImGui::Text("tangent"); ImGui::SameLine(); ImGui::InputDouble("tx", &c->tangent.x);
                                ImGui::SameLine(); ImGui::InputDouble("ty", &c->tangent.y);
        ImGui::InputDouble("normal force", &c->normal_force);
        ImGui::InputDouble("tangent force", &c->tangent_force);

        if (ImGui::Button("Delete")) {
            contacts.erase(contacts.begin() + settings.selected_contact);
            settings.selected_contact = -1;
        }
        
        ImGui::End();
    }

    if (settings.show_ids) {
        ImDrawList *draw_list = ImGui::GetForegroundDrawList();
        for (int i = 0; i < bodies.size(); i++) {
            Body *b = &bodies[i];
            
            int color = 0xFFFFFFFF;
            if (settings.selected_contact != -1) {
                Contact *c = &contacts[settings.selected_contact];
                if (c->i == i || c->j == i) color = 0xFF00FFFF;
            }

            draw_list->AddText(ImVec2(b->center.x, b->center.y), color, std::to_string(i).c_str());
        }
    }

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

Vec2 Vec2_Rotate(Vec2 v, float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

static bool PointInBody(Vec2 p, Body *b)
{
    Vec2 rel = Vec2_Sub(p, b->center);
    rel = Vec2_Rotate(rel, -b->angle);
    return fabs(rel.x) < b->size.x / 2 && fabs(rel.y) < b->size.y / 2;
}

void Render_OnMouse(int button, int action, int mods, double xpos, double ypos)
{
    if (mods & GLFW_MOD_SHIFT) { // select contact point
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            for (int i = 0; i < contacts.size(); i++) {
                if (fabs(contacts[i].pos.x - xpos) < 10 && fabs(contacts[i].pos.y - ypos) < 10
                    && settings.selected_contact != i) {
                    settings.selected_contact = i;
                    settings.selected_body = -1;
                    return;
                }
            }
        }
    }
    else { // select body
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            Vec2 p = {xpos, ypos};
            for (int i = 0; i < bodies.size(); i++) {
                if (PointInBody(p, &bodies[i]) && settings.selected_body != i) {
                    settings.selected_contact = -1;
                    settings.selected_body = i;
                    return;
                }
            }
        }
    }
}

void Render_OnKey(int key, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        settings.selected_body = -1;
    }

    if (settings.selected_body != -1) {
        Body *b = &bodies[settings.selected_body];
        
        double speed = (mods & GLFW_MOD_SHIFT ? 10 : 1);
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            switch (key) {
                case GLFW_KEY_W: b->center.y -= speed; break;
                case GLFW_KEY_S: b->center.y += speed; break;
                case GLFW_KEY_A: b->center.x -= speed; break;
                case GLFW_KEY_D: b->center.x += speed; break;
                case GLFW_KEY_Q: b->angle -= M_PI / 16.f; break;
                case GLFW_KEY_E: b->angle += M_PI / 16.f; break;
            }
        }
    }

    if (settings.selected_contact != -1) {
        Contact *c = &contacts[settings.selected_contact];
        
        double speed = (mods & GLFW_MOD_SHIFT ? 10 : 1);
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            switch (key) {
                case GLFW_KEY_W: c->pos.y -= speed; break;
                case GLFW_KEY_S: c->pos.y += speed; break;
                case GLFW_KEY_A: c->pos.x -= speed; break;
                case GLFW_KEY_D: c->pos.x += speed; break;
                case GLFW_KEY_Q: {
                    c->angle -= M_PI / 16.f;
                    c->normal.y = -cos(c->angle);
                    c->normal.x = sin(c->angle);
                    c->tangent.x = c->normal.y;
                    c->tangent.y = -c->normal.x;
                    break;
                }
                case GLFW_KEY_E: {
                    c->angle += M_PI / 16.f;
                    c->normal.y = -cos(c->angle);
                    c->normal.x = sin(c->angle);
                    c->tangent.x = c->normal.y;
                    c->tangent.y = -c->normal.x;
                    break;
                }
            }
        }
    }
}
