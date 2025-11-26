#include "render.h"

#include <glad/glad.h>
#include <cstdio>
#include <cmath>
#include <GLFW/glfw3.h>
#include <string>
#include <filesystem>

#include "linalg.h"
#include "physics.h"
#include "defines.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"
#include "imgui.h"

#define FOV 60

class FrameBuffer
{
public:
	FrameBuffer(float width, float height);
	~FrameBuffer();
	unsigned int getFrameTexture();
	void RescaleFrameBuffer(float width, float height);
	void Bind() const;
	void Unbind() const;
private:
	unsigned int fbo;
	unsigned int texture;
	unsigned int rbo;
};

struct Camera {
    Vec3 pos;
    float yaw, pitch;
    Vec3 forward, right, up;
};

enum class MoveMode {
    TRANSLATE,
    ROTATE,
    SCALE,
    NONE
};

struct OrthoView {
    Vec3 pos;
    double scale;
    const char *name;
    float view[16];
    FrameBuffer *fb;
    Vec3 axes[2];
};

struct Transform {
    MoveMode mode { MoveMode::NONE };
    Vec3 old_var;
    Vec3 *var;
    Vec2 start_mouse_pos;
};

std::vector<std::string> filenames;

static bool inited = false;
static OrthoView ortho_views[3];
FrameBuffer *fb_perspective = nullptr;
Transform cur_transform;

static Camera camera = {{0, 0, 0}, 0, 0};
static bool keys[512] = {false};

static double last_mouse_x, last_mouse_y;
static double mouse_dx, mouse_dy;

static bool first_mouse = true;
static bool right_mouse_down = false;
static bool left_mouse_down = false;

#define MENU_WIDTH 350
Vec2 view_size { (WIN_WIDTH - MENU_WIDTH)/2.f, WIN_HEIGHT/2.f };

struct Settings {
    float line_len { 0.1f };
    bool show_ids {true};
    bool show_axes { true };
    bool show_input_forces { false };
    bool show_output_forces { false };
    bool show_total_forces { false };
    bool show_rel_accel { true };

    char filename[128] { "" };
    int selected_body { -1 };
    int selected_contact { -1 };

    bool wireframe;
    
    double snap { 1 };
    bool snap_enabled { true };

    bool animate;
    double time = 0;

    bool less_arrows { false };
    bool show_input_rel_accel { false };

    bool highlight_selected { true };
};

static Settings settings;

void MakePerspectiveProj(float *res)
{
    float aspect = view_size.x / view_size.y;
    float fov = FOV * M_PI / 180.0f;
    float f = 1.0f / tan(fov / 2.0f);
    float near = 0.1f, far = 1000.0f;
    memset(res, 0, sizeof(float) * 16);
    res[0] = f/aspect;
    res[5] = f;
    res[10] = (far+near)/(near-far);
    res[11] = -1;
    res[14] = (2*far*near)/(near-far);
}

void Make3DView(float *res)
{
    Vec3 target = Vec3_Add(camera.pos, camera.forward);
    res[0] = camera.right.x; res[1] = camera.up.x; res[2] = -camera.forward.x; res[3] = 0;
    res[4] = camera.right.y; res[5] = camera.up.y; res[6] = -camera.forward.y; res[7] = 0;
    res[8] = camera.right.z; res[9] = camera.up.z; res[10] = -camera.forward.z; res[11] = 0;
    res[12] = -Vec3_Dot(camera.right, camera.pos);
    res[13] = -Vec3_Dot(camera.up, camera.pos);
    res[14] = Vec3_Dot(camera.forward, camera.pos);
    res[15] = 1;
}

void MakeOrthoProj(float *proj, float scale = 10)
{
    float aspect = view_size.x / view_size.y;
    float right = scale*aspect;
    float left = -scale*aspect;
    float top = scale;
    float bottom = -scale;
    float far = 1000;
    float near = -1000;
    memset(proj, 0, sizeof(float)*16);
    proj[0] = 2/(right-left);
    proj[3] = -(right+left)/(right-left);
    proj[5] = 2/(top-bottom);
    proj[7] = -(top+bottom)/(top-bottom);
    proj[10] = -2/(far-near);
    proj[11] = -(far+near)/(far-near);
    proj[15] = 1;
}

static Vec3 SnapCoords(Vec3 coords, float s)
{
    if (settings.snap_enabled)
        return { roundf(coords.x / s) * s, roundf(coords.y / s) * s, roundf(coords.z / s) * s };
    return coords;
}

static Vec3 SnapCoords(Vec3 coords)
{
    return SnapCoords(coords, settings.snap);
}

static Vec3 ViewToWorld(Vec3 view_dir)
{
    return Vec3_Normalize((Vec3){
        camera.right.x * view_dir.x + camera.up.x * view_dir.y - camera.forward.x * view_dir.z,
        camera.right.y * view_dir.x + camera.up.y * view_dir.y - camera.forward.y * view_dir.z,
        camera.right.z * view_dir.x + camera.up.z * view_dir.y - camera.forward.z * view_dir.z
    });
}

static Vec3 WorldToClip(Vec3 world, const float *view, const float *proj)
{
    float vx = view[0] * world.x + view[4] * world.y + view[8] * world.z + view[12];
    float vy = view[1] * world.x + view[5] * world.y + view[9] * world.z + view[13];
    float vz = view[2] * world.x + view[6] * world.y + view[10] * world.z + view[14];
    float vw = view[3] * world.x + view[7] * world.y + view[11] * world.z + view[15];
    return {
        proj[0] * vx + proj[4] * vy + proj[8] * vz + proj[12] * vw,
        proj[1] * vx + proj[5] * vy + proj[9] * vz + proj[13] * vw,
        proj[3] * vx + proj[7] * vy + proj[11] * vz + proj[15] * vw
    };
}

static Vec3 ClipToScreen(Vec3 clip)
{
    return {
        MENU_WIDTH + (clip.x / clip.z + 1.0f) * 0.5f * view_size.x,
        (1.0f - clip.y / clip.z) * 0.5f * view_size.y,
        0.0f
    };
}

#define VIEW_NONE -2
#define VIEW_PERSPECTIVE -1
#define VIEW_ORTHO 0

int GetViewIdx()
{
    if (last_mouse_x > MENU_WIDTH && last_mouse_x < MENU_WIDTH+view_size.x) {
        if (last_mouse_y > view_size.y) return VIEW_ORTHO + 1;
        return VIEW_PERSPECTIVE;
    }
    else if (last_mouse_x > MENU_WIDTH+view_size.x) {
        if (last_mouse_y > view_size.y) return VIEW_ORTHO + 2;
        return VIEW_ORTHO + 0;
    }
    return VIEW_NONE;
}

Vec3 Matrix4x4MulVec3(float *mat, Vec3 vec)
{
    double w = mat[3] * vec.x + mat[7] * vec.y + mat[11] * vec.z + mat[15];
    return {
        (mat[0] * vec.x + mat[4] * vec.y + mat[8] * vec.z + mat[12]) / w,
        (mat[1] * vec.x + mat[5] * vec.y + mat[9] * vec.z + mat[13]) / w,
        (mat[2] * vec.x + mat[6] * vec.y + mat[10] * vec.z + mat[14]) / w
    };
}

static bool Click3DPoint(Vec3 point, double *dist, double radius = 0.5)
{
    int idx = GetViewIdx();
    if (idx != VIEW_PERSPECTIVE) return false;
    
    double mx = fmod(last_mouse_x - MENU_WIDTH, view_size.x);
    double my = fmod(last_mouse_y, view_size.y);

    float proj[16];
    float view_mat[16];
    MakePerspectiveProj(proj);
    Make3DView(view_mat);
    Vec3 ndc_point = Matrix4x4MulVec3(proj, Matrix4x4MulVec3(view_mat, point));
    ndc_point.z = 0;

    // TODO: calc once per frame
    Vec3 ndc_mouse = {
        mx / view_size.x * 2 - 1,
        -(my / view_size.y * 2 - 1),
        0
    };

    printf("ndc_point: %lf %lf %lf\n", ndc_point.x, ndc_point.y, ndc_point.z);
    printf("ndc_mouse: %lf %lf %lf\n", ndc_mouse.x, ndc_mouse.y, ndc_mouse.z);

    // TODO: make dist^2
    *dist = Vec3_Length(Vec3_Sub(ndc_mouse, ndc_point));
    return *dist < radius;
}

static void DrawArrow(Vec3 p, Vec3 dir)
{
    glLineWidth(3.f);
    glBegin(GL_LINES);
    glVertex3f(p.x, p.y, p.z);
    glVertex3f(p.x + dir.x * settings.line_len,
               p.y + dir.y * settings.line_len,
               p.z + dir.z * settings.line_len);
    glEnd();
}

static void UpdateCameraVectors()
{
    camera.forward.x = cos(camera.yaw) * cos(camera.pitch);
    camera.forward.y = sin(camera.pitch);
    camera.forward.z = sin(camera.yaw) * cos(camera.pitch);
    camera.forward = Vec3_Normalize(camera.forward);
    camera.right = Vec3_Normalize(Vec3_Cross(camera.forward, {0, 1, 0}));
    camera.up = Vec3_Normalize(Vec3_Cross(camera.right, camera.forward));
}

void MoveCamera()
{
    if (!right_mouse_down) {
        return;
    }

    int idx = GetViewIdx();
    if (idx == VIEW_PERSPECTIVE) {
        float sensitivity = 0.003f;
        camera.yaw += mouse_dx * sensitivity;
        camera.pitch -= mouse_dy * sensitivity;

        if (camera.pitch > M_PI / 2 - 0.01f) camera.pitch = M_PI / 2 - 0.01f;
        if (camera.pitch < -M_PI / 2 + 0.01f) camera.pitch = -M_PI / 2 + 0.01f;
    }
    else {
        OrthoView &v = ortho_views[idx];
        v.pos.x -= mouse_dx * 0.05f * v.scale;
        v.pos.y += mouse_dy * 0.05f * v.scale;
    }
}

void DrawGrid(Vec3 a1, Vec3 a2, Vec3 center, int a1_len = 10, int a2_len = 10, double gap = 1)
{
    a1 = Vec3_Scale(a1, gap);
    a2 = Vec3_Scale(a2, gap);
    a1_len = (double)a1_len / gap;
    a2_len = (double)a2_len / gap;

    glLineWidth(1.f);
    glColor4f(1.f, 1.f, 1.f, 0.1f);
    glBegin(GL_LINES);
    for (int i = -a1_len; i < a1_len; i++) {
        glVertex3f(center.x + a1.x*i + a2.x*a2_len, center.y + a1.y*i + a2.y*a2_len, center.z + a1.z*i + a2.z*a2_len);
        glVertex3f(center.x + a1.x*i - a2.x*a2_len, center.y + a1.y*i - a2.y*a2_len, center.z + a1.z*i - a2.z*a2_len);
    }
    for (int i = -a2_len; i < a2_len; i++) {
        glVertex3f(center.x + a2.x*i + a1.x*a1_len, center.y + a2.y*i + a1.y*a1_len, center.z + a2.z*i + a1.z*a1_len);
        glVertex3f(center.x + a2.x*i - a1.x*a1_len, center.y + a2.y*i - a1.y*a1_len, center.z + a2.z*i - a1.z*a1_len);
    }
    glEnd();
}

void MoveBody()
{
    if (cur_transform.mode == MoveMode::NONE) return;

    int view_idx = GetViewIdx();
    if (view_idx == VIEW_PERSPECTIVE) return;
    OrthoView &v = ortho_views[view_idx];
    Vec3 a1 = v.axes[0];
    Vec3 a2 = v.axes[1];

    double mul = 0.05;
    if (cur_transform.mode == MoveMode::SCALE) mul *= 2.f;

    double dx = (last_mouse_x - cur_transform.start_mouse_pos.x) * mul;
    double dy = (last_mouse_y - cur_transform.start_mouse_pos.y) * -mul;

    if (keys[GLFW_KEY_LEFT_SHIFT] == GLFW_PRESS) {
        if (fabs(dx) > fabs(dy)) dy = 0;
        else dx = 0;
    }

    
    if (cur_transform.var != nullptr && cur_transform.mode != MoveMode::ROTATE) {
        *cur_transform.var = Vec3_Add(cur_transform.old_var,
            Vec3_Add(Vec3_Scale(a1, dx * v.scale), Vec3_Scale(a2, dy * v.scale)));
        *cur_transform.var = SnapCoords(*cur_transform.var);
    }
    else if (cur_transform.var != nullptr) {
        Vec3 a = Vec3_Sub({1, 1, 1}, Vec3_Add(a1, a2));
        *cur_transform.var = Vec3_Add(cur_transform.old_var, Vec3_Scale(a, dx));
    }
}

void Render_UpdateCamera(double dt)
{
    if (!right_mouse_down) return;

    float speed = 5.0f * dt;
    if (keys[GLFW_KEY_LEFT_SHIFT]) speed *= 3.0f;
    
    if (keys[GLFW_KEY_W]) camera.pos = Vec3_Add(camera.pos, Vec3_Scale(camera.forward, speed));
    if (keys[GLFW_KEY_S]) camera.pos = Vec3_Sub(camera.pos, Vec3_Scale(camera.forward, speed));
    if (keys[GLFW_KEY_A]) camera.pos = Vec3_Sub(camera.pos, Vec3_Scale(camera.right, speed));
    if (keys[GLFW_KEY_D]) camera.pos = Vec3_Add(camera.pos, Vec3_Scale(camera.right, speed));

    if (settings.animate) {
        settings.time += dt;
    }
}

static void DrawBoxShaded(Vec3 center, Vec3 size, Vec3 euler, Vec3 color)
{
    Vec3 sun_dir = Vec3_Normalize({ 2, 3, 1 });
    
    float cx = cos(euler.x * M_PI / 180), sx = sin(euler.x * M_PI / 180);
    float cy = cos(euler.y * M_PI / 180), sy = sin(euler.y * M_PI / 180);
    float cz = cos(euler.z * M_PI / 180), sz = sin(euler.z * M_PI / 180);
    Vec3 xpos_dir = {cy*cz, cy*sz, -sy};
    Vec3 ypos_dir = {sx*sy*cz - cx*sz, sx*sy*sz + cx*cz, sx*cy};
    Vec3 zpos_dir = {cx*sy*cz + sx*sz, cx*sy*sz - sx*cz, cx*cy};
    Vec3 xneg_dir = {-xpos_dir.x, -xpos_dir.y, -xpos_dir.z};
    Vec3 yneg_dir = {-ypos_dir.x, -ypos_dir.y, -ypos_dir.z};
    Vec3 zneg_dir = {-zpos_dir.x, -zpos_dir.y, -zpos_dir.z};

    float xpos_light = fmax(0.3f, Vec3_Dot(xpos_dir, sun_dir));
    float ypos_light = fmax(0.3f, Vec3_Dot(ypos_dir, sun_dir));
    float zpos_light = fmax(0.3f, Vec3_Dot(zpos_dir, sun_dir));
    float xneg_light = fmax(0.3f, Vec3_Dot(xneg_dir, sun_dir));
    float yneg_light = fmax(0.3f, Vec3_Dot(yneg_dir, sun_dir));
    float zneg_light = fmax(0.3f, Vec3_Dot(zneg_dir, sun_dir));
    
    Vec3 xpos_col = {color.x * xpos_light, color.y * xpos_light, color.z * xpos_light};
    Vec3 ypos_col = {color.x * ypos_light, color.y * ypos_light, color.z * ypos_light};
    Vec3 zpos_col = {color.x * zpos_light, color.y * zpos_light, color.z * zpos_light};
    Vec3 xneg_col = {color.x * xneg_light, color.y * xneg_light, color.z * xneg_light};
    Vec3 yneg_col = {color.x * yneg_light, color.y * yneg_light, color.z * yneg_light};
    Vec3 zneg_col = {color.x * zneg_light, color.y * zneg_light, color.z * zneg_light};

    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    glRotatef(euler.x, 1, 0, 0);
    glRotatef(euler.y, 0, 1, 0);
    glRotatef(euler.z, 0, 0, 1);

    float x = size.x / 2, y = size.y / 2, z = size.z / 2;
    glBegin(GL_QUADS);
    glColor3f(zneg_col.x, zneg_col.y, zneg_col.z);
    glVertex3f(-x, -y, -z); glVertex3f(+x, -y, -z); glVertex3f(+x, +y, -z); glVertex3f(-x, +y, -z);
    glColor3f(zpos_col.x, zpos_col.y, zpos_col.z);
    glVertex3f(-x, -y, +z); glVertex3f(+x, -y, +z); glVertex3f(+x, +y, +z); glVertex3f(-x, +y, +z);
    glColor3f(xneg_col.x, xneg_col.y, xneg_col.z);
    glVertex3f(-x, -y, -z); glVertex3f(-x, -y, +z); glVertex3f(-x, +y, +z); glVertex3f(-x, +y, -z);
    glColor3f(xpos_col.x, xpos_col.y, xpos_col.z);
    glVertex3f(+x, -y, -z); glVertex3f(+x, -y, +z); glVertex3f(+x, +y, +z); glVertex3f(+x, +y, -z);
    glColor3f(yneg_col.x, yneg_col.y, yneg_col.z);
    glVertex3f(-x, -y, -z); glVertex3f(+x, -y, -z); glVertex3f(+x, -y, +z); glVertex3f(-x, -y, +z);
    glColor3f(ypos_col.x, ypos_col.y, ypos_col.z);
    glVertex3f(-x, +y, -z); glVertex3f(+x, +y, -z); glVertex3f(+x, +y, +z); glVertex3f(-x, +y, +z);
    glEnd();
    
    glPopMatrix();
}

static void DrawBoxWireframe(Vec3 center, Vec3 size, Vec3 euler, Vec3 color)
{
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    glRotatef(euler.x, 1, 0, 0);
    glRotatef(euler.y, 0, 1, 0);
    glRotatef(euler.z, 0, 0, 1);

    float x = size.x / 2, y = size.y / 2, z = size.z / 2;
    glColor3f(color.x, color.y, color.z);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-x, -y, -z); glVertex3f(+x, -y, -z); glVertex3f(+x, +y, -z); glVertex3f(-x, +y, -z);
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    glVertex3f(-x, -y, +z); glVertex3f(+x, -y, +z); glVertex3f(+x, +y, +z); glVertex3f(-x, +y, +z);
    glEnd();
    
    glBegin(GL_LINES);
    glVertex3f(-x, -y, -z);
    glVertex3f(-x, -y, +z);
    glVertex3f(-x, +y, -z);
    glVertex3f(-x, +y, +z);
    glVertex3f(+x, -y, -z);
    glVertex3f(+x, -y, +z);
    glVertex3f(+x, +y, -z);
    glVertex3f(+x, +y, +z);
    glEnd();

    glPopMatrix();
}

static void DrawBox(Vec3 center, Vec3 size, Vec3 euler, Vec3 color)
{
    if (settings.wireframe)
        DrawBoxWireframe(center, size, euler, color);
    else
        DrawBoxShaded(center, size, euler, color);
}

Vec3 GetBodyAccel(int i)
{
    Vec3 accel = bodies[i].accel;
    double mass = bodies[i].mass;

    for (int j = 0; j < contacts.size(); j++) {
        Contact &c = contacts[j];
        if (c.j == i) {
            accel = Vec3_Add(accel, Vec3_Scale(c.axes[AXIS_NORMAL], c.res_normal_force / mass));
            for (int k = 0; k < N_TANGENTS; k++)
                accel = Vec3_Add(accel, Vec3_Scale(c.axes[AXIS_TANGENT1 + k], c.res_tangent_force[k] / mass));
        }
        else if (c.i == i) {
            accel = Vec3_Sub(accel, Vec3_Scale(c.axes[AXIS_NORMAL], c.res_normal_force / mass));
            for (int k = 0; k < N_TANGENTS; k++)
                accel = Vec3_Sub(accel, Vec3_Scale(c.axes[AXIS_TANGENT1 + k], c.res_tangent_force[k] / mass));
        }
    }
    return accel;
}

#define N_BODY_COLORS 16
static Vec3 body_colors[N_BODY_COLORS] = {
    {0.7, 0.5, 0.5}, {0.5, 0.7, 0.5}, {0.5, 0.5, 0.7}, {0.7, 0.7, 0.5}, {0.5, 0.7, 0.7}, {0.7, 0.5, 0.7},
    {0.6, 0.5, 0.5}, {0.5, 0.6, 0.5}, {0.5, 0.5, 0.6}, {0.6, 0.6, 0.5}, {0.5, 0.6, 0.6}, {0.6, 0.5, 0.6},
    {0.6, 0.6, 0.6}, {0.65, 0.6, 0.55}, {0.6, 0.65, 0.55}, {0.55, 0.6, 0.65}
};

FrameBuffer::FrameBuffer(float width, float height)
{
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        printf("framebuffer is not complete!\n");
    
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

FrameBuffer::~FrameBuffer()
{
	glDeleteFramebuffers(1, &fbo);
	glDeleteTextures(1, &texture);
	glDeleteRenderbuffers(1, &rbo);
}

unsigned int FrameBuffer::getFrameTexture()
{
	return texture;
}

void FrameBuffer::RescaleFrameBuffer(float width, float height)
{
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
}

void FrameBuffer::Bind() const
{
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
}

void FrameBuffer::Unbind() const
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Show_View(FrameBuffer *fb, int px, int py, const char *name)
{
    ImGui::SetNextWindowPos(ImVec2(MENU_WIDTH + px*view_size.x, + py*view_size.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2((WIN_WIDTH - MENU_WIDTH) / 2.f, WIN_HEIGHT / 2.f), ImGuiCond_Always);
    ImGui::Begin(name, NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);
    ImGui::SetNextItemWidth((WIN_WIDTH - MENU_WIDTH) / 2.f);
    ImGui::BeginChild((std::string("render_") + name).c_str());	
    ImGui::Image(
        (ImTextureID)fb->getFrameTexture(), 
        ImGui::GetContentRegionAvail(), 
        ImVec2(0, 1), ImVec2(1, 0)
    );
    ImGui::EndChild();
    ImGui::End();
}

void Render_Scene(FrameBuffer *fb, float proj[16], float view[16])
{
    glViewport(0, 0, view_size.x, view_size.y);
    glClearColor(33.0f / 255.f, 37.0f / 255.f, 43.0f / 255.f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    UpdateCameraVectors();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_MULTISAMPLE);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glLoadMatrixf(proj);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrixf(view);
    
    for (int i = 0; i < bodies.size(); i++) {
        Vec3 color = body_colors[i % N_BODY_COLORS];
        if (i == settings.selected_body && settings.highlight_selected) {
            color = {1, 1, 0};
        }

        Vec3 center = bodies[i].center;
        if (settings.animate) {
            center = Vec3_Add(center, Vec3_Scale(GetBodyAccel(i), settings.time*settings.time*0.5));
        }

        DrawBox(center, bodies[i].size, bodies[i].euler, color);
    }

    glClear(GL_DEPTH_BUFFER_BIT);

    for (Contact &c : contacts) {
        float cx = cos(c.angles.x * M_PI / 180), sx = sin(c.angles.x * M_PI / 180);
        float cy = cos(c.angles.y * M_PI / 180), sy = sin(c.angles.y * M_PI / 180);
        float cz = cos(c.angles.z * M_PI / 180), sz = sin(c.angles.z * M_PI / 180);
        c.axes[AXIS_NORMAL].x = sx*sy*cz - cx*sz;
        c.axes[AXIS_NORMAL].y = sx*sy*sz + cx*cz;
        c.axes[AXIS_NORMAL].z = sx*cy;

        Vec3 T0 = {cy*cz, cy*sz, -sy};
        Vec3 T1 = Vec3_Cross(T0, c.axes[AXIS_NORMAL]);
        
        for (int i = 0; i < N_TANGENTS; i++) {
            double alpha = i * M_PI / (double)N_TANGENTS;
            double cs = cos(alpha), sn = sin(alpha);
            c.axes[AXIS_TANGENT1 + i] = Vec3_Add(Vec3_Scale(T0, cs), Vec3_Scale(T1, sn));
        }

        for (int i = 0; i < N_TANGENTS+1; i++) {
            c.axes[i] = Vec3_Normalize(c.axes[i]);
        }
    }

    glLineWidth(1.f);
    if (settings.show_input_forces) {
        glColor3f(1.f, 0.3f, 0.3f);
        for (int i = 0; i < bodies.size(); i++) {
            if (settings.less_arrows && i != settings.selected_body) continue;
            Body *b = &bodies[i];
            if (Vec3_Length(b->accel) > 1e-6)
                DrawArrow(b->center, b->accel);
        }
    }

    if (settings.show_input_rel_accel) {
        glColor3f(0.3f, 1.f, 0.3f);
        for (int i = 0; i < contacts.size(); i++) {
            if (settings.less_arrows && i != settings.selected_contact) continue;
            Contact *c = &contacts[i];
            Vec3 rel_accel = Vec3_Sub(bodies[c->j].accel, bodies[c->i].accel);
            if (Vec3_Length(rel_accel) > 1e-6)
                DrawArrow(c->pos, rel_accel);
        }
    }

    if (settings.show_output_forces) {
        glColor3f(1.f, 1.f, 0.3f);
        for (int i = 0; i < contacts.size(); i++) {
            if (settings.less_arrows && i != settings.selected_contact) continue;
            Contact *c = &contacts[i];
            if (fabs(c->res_normal_force) > 1e-4)
                DrawArrow(c->pos, Vec3_Scale(c->axes[AXIS_NORMAL], c->res_normal_force));

            for (int j = 0; j < N_TANGENTS; j++) {
                if (fabs(c->res_tangent_force[j]) > 1e-4) {
                    DrawArrow(c->pos, Vec3_Scale(c->axes[AXIS_TANGENT1 + j], c->res_tangent_force[j]));
                }
            }
        }
    }

    if (settings.show_total_forces) {
        glColor3f(1.f, 1.f, 1.f);
        for (int i = 0; i < bodies.size(); i++) {
            if (settings.less_arrows && i != settings.selected_body) continue;
            Body *b = &bodies[i];
            Vec3 accel = GetBodyAccel(i);
            if (Vec3_Length(accel) > 1e-6f) DrawArrow(b->center, accel);
        }
    }

    if (settings.show_rel_accel) {
        glColor3f(1.f, 1.f, 1.f);
        for (int i = 0; i < contacts.size(); i++) {
            if (settings.less_arrows && i != settings.selected_contact) continue;
            Contact &c = contacts[i];
            Vec3 i_accel = GetBodyAccel(c.i);
            Vec3 j_accel = GetBodyAccel(c.j);
            Vec3 diff = Vec3_Sub(j_accel, i_accel);
            if (Vec3_Length(diff) > 1e-6f)
                DrawArrow(c.pos, diff);
        }
    }

    if (settings.show_axes) {
        for (int i = 0; i < contacts.size(); i++) {
            if (settings.selected_contact == i && settings.highlight_selected) {
                glColor3f(1.f, 1.f, 0.f);
                glLineWidth(4.f);
            }
            else {
                glColor3f(0.3f, 0.3f, 1.f);
                glLineWidth(2.f);
            }
            
            Contact *c = &contacts[i];
            glBegin(GL_LINES);
            for (int j = 0; j < N_TANGENTS+1; j++) {
                Vec3 v = Vec3_Scale(c->axes[j], 0.25);
                glVertex3f(c->pos.x, c->pos.y, c->pos.z);
                glVertex3f(c->pos.x + v.x, c->pos.y + v.y, c->pos.z + v.z);
            }
            glEnd();
        }
    }
}

void Render_Perspective()
{
    settings.wireframe = false;

    float proj[16], view[16];
    MakePerspectiveProj(proj);
    Make3DView(view);

    fb_perspective->Bind();
    Render_Scene(fb_perspective, proj, view);

    if (settings.show_ids) {
        ImDrawList *draw_list = ImGui::GetForegroundDrawList();
        for (int i = 0; i < bodies.size(); i++) {
            Vec3 clip = WorldToClip(bodies[i].center, view, proj);
            if (clip.z > 0) {
                Vec3 screen = ClipToScreen(clip);
                if (screen.x >= MENU_WIDTH && screen.y >= 0 && screen.x < MENU_WIDTH+view_size.x && screen.y < view_size.y) {
                    std::string s = std::to_string(i);
                    ImVec2 sz = ImGui::CalcTextSize(s.c_str());
                    draw_list->AddText(ImVec2(screen.x - sz.x/2.0, screen.y - sz.y/2.0), 0xFFFFFFFF, s.c_str());
                }
            }
        }
    }

    fb_perspective->Unbind();
}

void Render_Ortho(OrthoView &view)
{
    float proj[16];
    MakeOrthoProj(proj, 10.0 * view.scale);
    settings.wireframe = true;

    float view_mat[16];
    memcpy(view_mat, view.view, sizeof(view_mat));
    view_mat[12] = -view.pos.x;
    view_mat[13] = -view.pos.y;

    view.fb->Bind();
    Render_Scene(view.fb, proj, view_mat);
    DrawGrid(view.axes[0], view.axes[1],
        SnapCoords(
            Vec3_Add(Vec3_Scale(view.axes[0], view.pos.x), Vec3_Scale(view.axes[1], view.pos.y)),
            settings.snap_enabled ? settings.snap : 1
        ),
        20 * view.scale, 20 * view.scale,
        settings.snap_enabled ? settings.snap : 1
    );
    view.fb->Unbind();
}

void GetFilenames()
{
    filenames.clear();
    try {
        std::string path = "./examples";
        for (const auto& entry : std::filesystem::directory_iterator(path)) {
            filenames.push_back("examples/" + entry.path().filename().string());
        }
    } catch(std::exception &e) {
        // ...
    }
}

void DeleteBody()
{
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

void DeleteContact()
{
    contacts.erase(contacts.begin() + settings.selected_contact);
    settings.selected_contact = -1;
}

void Render_Draw()
{
    if (!inited) {
        inited = true;
        
        ortho_views[0] = OrthoView{
            {0, 0}, 1, "XY",
            {
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
            },
            new FrameBuffer(view_size.x, view_size.y),
            {{1, 0, 0}, {0, 1, 0}}
        };

        ortho_views[1] = OrthoView{
            {0, 0}, 1, "XZ",
            {
                1, 0, 0, 0,
                0, 0, 1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1
            },
            new FrameBuffer(view_size.x, view_size.y),
            {{1, 0, 0}, {0, 0, 1}}
        };

        ortho_views[2] = OrthoView{
            {0, 0}, 1, "YZ",
            {
                0, 0, 1, 0,
                0, 1, 0, 0,
                1, 0, 0, 0,
                0, 0, 0, 1
            },
            new FrameBuffer(view_size.x, view_size.y),
            {{0, 0, 1}, {0, 1, 0}}
        };
        
        fb_perspective = new FrameBuffer(view_size.x, view_size.y);
        GetFilenames();
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    Render_Perspective();
    Render_Ortho(ortho_views[0]);
    Render_Ortho(ortho_views[1]);
    Render_Ortho(ortho_views[2]);
    
    Show_View(fb_perspective, 0, 0, "persp");
    Show_View(ortho_views[0].fb, 1, 0, ortho_views[0].name);
    Show_View(ortho_views[1].fb, 0, 1, ortho_views[1].name);
    Show_View(ortho_views[2].fb, 1, 1, ortho_views[2].name);

    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(MENU_WIDTH, WIN_HEIGHT), ImGuiCond_Always);
    ImGui::Begin("Settings", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
    if (ImGui::CollapsingHeader("Help")) {
        ImGui::SeparatorText("3D view");
        ImGui::Text("Right mouse button: look around");
        ImGui::Text("  + WASD: move");
        ImGui::Text("Left click: select body");
        ImGui::Text("Shift + Left click: select contact point");
        
        ImGui::SeparatorText("2D view");
        ImGui::Text("Right mouse button: move around");
        ImGui::Text("Left mouse button: manipulate objects");
        ImGui::Text("  + Shift: snap to the grid");
        ImGui::Text("  + Alt: scale selected body");
        ImGui::Text("  + Ctrl: rotate selected object");

        ImGui::SeparatorText("Menu");
        ImGui::Text("Self-explanatory");
    }
    if (ImGui::CollapsingHeader("Show")) {
        ImGui::Checkbox("Ids", &settings.show_ids); ImGui::SameLine();
        ImGui::Checkbox("Axes", &settings.show_axes);
        
        ImGui::Text("Forces"); ImGui::SameLine();
        ImGui::Checkbox("Input", &settings.show_input_forces); ImGui::SameLine();
        ImGui::Checkbox("Output", &settings.show_output_forces); ImGui::SameLine();
        ImGui::Checkbox("Total", &settings.show_total_forces);

        ImGui::Text("Rel accel"); ImGui::SameLine();
        ImGui::Checkbox("Input##in_rel", &settings.show_input_rel_accel); ImGui::SameLine();
        ImGui::Checkbox("Result", &settings.show_rel_accel);

        ImGui::Checkbox("Less arrows", &settings.less_arrows);
        ImGui::Checkbox("Highlight selected", &settings.highlight_selected);
    }
    if (ImGui::CollapsingHeader("File")) {
        ImGui::InputText("Filename", settings.filename, sizeof(settings.filename));
    
        ImGui::BeginListBox("Examples");
        for (std::string &filename : filenames) {
            if (ImGui::Selectable(filename.c_str(), settings.filename == filename)) {
                strcpy(settings.filename, filename.c_str());
            }
        }
        ImGui::EndListBox();

        if (ImGui::Button("Load")) {
            Phys_ReadFromFile(settings.filename);
        }
        ImGui::SameLine();
        if (ImGui::Button("Save") && bodies.size() > 0) {
            Phys_SaveToFile(settings.filename);
            GetFilenames();
        }
        ImGui::SameLine();
        if (ImGui::Button("New")) {
            contacts.clear();
            bodies.clear();
        }
    }

    if (ImGui::Button("Solve")) {
        Phys_Solve();
    }

    ImGui::InputFloat("Line length", &settings.line_len, 0.1f, 1.f, "%.3f");
    if (settings.snap_enabled) {
        ImGui::Text("Snap: %lf", settings.snap); ImGui::SameLine();
        if (ImGui::Button("+")) settings.snap *= 2; ImGui::SameLine();
        if (ImGui::Button("-")) settings.snap /= 2; ImGui::SameLine();
        if (ImGui::Button("OFF")) settings.snap_enabled = false;
    }
    else {
        ImGui::Text("Snap: OFF"); ImGui::SameLine();
        if (ImGui::Button("ON")) settings.snap_enabled = true;
    }

    if (settings.animate) {
        if (ImGui::Button("Stop")) settings.animate = false;
    }
    else {
        if (ImGui::Button("Play")) {
            settings.animate = true;
            settings.time = 0;
        }
    }

    ImGui::Separator();
    if (ImGui::Button("New contact point")) {
        Vec3 spawn_pos = Vec3_Add(camera.pos, Vec3_Scale(camera.forward, 5.0));
        Contact new_contact;
        memset(&new_contact, 0, sizeof(Contact));
        new_contact.mu = 1.0;
        new_contact.pos = spawn_pos;
        contacts.push_back(new_contact);
    }
    if (ImGui::Button("New body")) {
        Vec3 spawn_pos = Vec3_Add(camera.pos, Vec3_Scale(camera.forward, 5.0));
        if (settings.selected_body != -1) {
            Body new_body = bodies[settings.selected_body];
            new_body.center = spawn_pos;
            bodies.push_back(new_body);
        } 
        else {
            bodies.push_back(Body {
                1.0, {0.0, 0.0, 0.0}, spawn_pos, {2.0, 1.0, 2.0}, {0.0, 0.0, 0.0}
            });
        }
    }
    

    if (settings.selected_body != -1) {
        ImGui::Separator();
        ImGui::Text("Body %d", settings.selected_body);
        Body *b = &bodies[settings.selected_body];
        ImGui::InputDouble("Mass", &b->mass); ImGui::SameLine();
        if (ImGui::Button("INF")) {
            b->mass = INFINITY;
        }

        ImGui::Columns(4);
        ImGui::Text("pos"); ImGui::NextColumn();
        ImGui::InputDouble("##px", &b->center.x); ImGui::NextColumn();
        ImGui::InputDouble("##py", &b->center.y); ImGui::NextColumn();
        ImGui::InputDouble("##pz", &b->center.z); ImGui::NextColumn();
        ImGui::Text("accel"); ImGui::NextColumn();
        ImGui::InputDouble("##ax", &b->accel.x); ImGui::NextColumn();
        ImGui::InputDouble("##ay", &b->accel.y); ImGui::NextColumn();
        ImGui::InputDouble("##az", &b->accel.z); ImGui::NextColumn();
        ImGui::Text("size"); ImGui::NextColumn();
        ImGui::InputDouble("##sx", &b->size.x); ImGui::NextColumn();
        ImGui::InputDouble("##sy", &b->size.y); ImGui::NextColumn();
        ImGui::InputDouble("##sz", &b->size.z); ImGui::NextColumn();
        ImGui::Text("euler"); ImGui::NextColumn();
        ImGui::InputDouble("##ex", &b->euler.x); ImGui::NextColumn();
        ImGui::InputDouble("##ey", &b->euler.y); ImGui::NextColumn();
        ImGui::InputDouble("##ez", &b->euler.z); ImGui::NextColumn();
        ImGui::Columns();
        
        if (ImGui::Button("G")) {
            b->accel = {0.0, -9.8, 0.0};
        }

        if (ImGui::Button("Delete"))
            DeleteBody();
    }

    if (settings.selected_contact != -1) {
        ImGui::Separator();
        ImGui::Text("Contact %d", settings.selected_contact);
        Contact *c = &contacts[settings.selected_contact];

        ImGui::InputInt("Body i", &c->i);
        ImGui::InputInt("Body j", &c->j);
        ImGui::InputDouble("Mu", &c->mu);

        ImGui::Columns(4, "my_grid_columns");        
        ImGui::Text("pos"); ImGui::NextColumn();
        ImGui::InputDouble("##px", &c->pos.x); ImGui::NextColumn();
        ImGui::InputDouble("##py", &c->pos.y); ImGui::NextColumn();
        ImGui::InputDouble("##pz", &c->pos.z); ImGui::NextColumn();
        ImGui::Text("n"); ImGui::NextColumn();
        ImGui::InputDouble("##nx", &c->axes[AXIS_NORMAL].x); ImGui::NextColumn();
        ImGui::InputDouble("##ny", &c->axes[AXIS_NORMAL].y); ImGui::NextColumn();
        ImGui::InputDouble("##nz", &c->axes[AXIS_NORMAL].z); ImGui::NextColumn();
        ImGui::Text("euler"); ImGui::NextColumn();
        ImGui::InputDouble("##ec1", &c->angles.x); ImGui::NextColumn();
        ImGui::InputDouble("##ec2", &c->angles.y); ImGui::NextColumn();
        ImGui::InputDouble("##ec3", &c->angles.z); ImGui::NextColumn();
        
        ImGui::Columns();
        ImGui::InputDouble("normal force", &c->res_normal_force);
        double sum = 0;
        for (int i = 0; i < N_TANGENTS; i++) {
            sum += fabs(c->res_tangent_force[i]);
        }
        ImGui::Text("tangent forces sum: %lf", sum);

        if (ImGui::Button("Delete"))
            DeleteContact();
    }
    
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());


    // mouse shit
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    
    if (first_mouse) {
        last_mouse_x = xpos;
        last_mouse_y = ypos;
        first_mouse = false;
        return;
    }

    mouse_dx = xpos - last_mouse_x;
    mouse_dy = ypos - last_mouse_y;
    last_mouse_x = xpos;
    last_mouse_y = ypos;

    MoveCamera();
    MoveBody();
}

bool SelectBody(int button, int action, int mods)
{
    if (last_mouse_x < MENU_WIDTH) return false;

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        bool is_body = false;
        int closest = -1;
        float closest_dist = 1e9;
        for (int i = 0; i < contacts.size(); i++) {
            double dist;
            if (Click3DPoint(contacts[i].pos, &dist)) {
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest = i;
                }
            }
        }
        for (int i = 0; i < bodies.size(); i++) {
            double dist;
            if (Click3DPoint(bodies[i].center, &dist)) {
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest = i;
                    is_body = true;
                }
            }
        }
        if (closest != -1) {
            if (is_body) { settings.selected_body = closest; settings.selected_contact = -1; }
            else { settings.selected_contact = closest; settings.selected_body = -1; }
        }
        return true;
    }
    return false;
}

void BeginEndTransform(int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        cur_transform.mode = MoveMode::NONE;
        return;
    }

    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS)
        return;

    MoveMode cur_mode = MoveMode::TRANSLATE;
    if (keys[GLFW_KEY_LEFT_ALT]) {
        cur_mode = MoveMode::SCALE;
    }
    else if (keys[GLFW_KEY_LEFT_CONTROL]) {
        cur_mode = MoveMode::ROTATE;
    }

    SelectBody(button, action, mods);

    if (settings.selected_body != -1) {
        switch (cur_mode) {
        case MoveMode::SCALE:
            cur_transform.old_var = bodies[settings.selected_body].size;
            cur_transform.var = &bodies[settings.selected_body].size;
            break;
        case MoveMode::TRANSLATE:
            cur_transform.old_var = bodies[settings.selected_body].center;
            cur_transform.var = &bodies[settings.selected_body].center;
            break;
        case MoveMode::ROTATE:
            cur_transform.old_var = bodies[settings.selected_body].euler;
            cur_transform.var = &bodies[settings.selected_body].euler;
            break;
        case MoveMode::NONE:
            return;
        }
    }
    else if (settings.selected_contact != -1) {
        switch (cur_mode) {
        case MoveMode::TRANSLATE:
            cur_transform.old_var = contacts[settings.selected_contact].pos;
            cur_transform.var = &contacts[settings.selected_contact].pos;
            break;
        case MoveMode::ROTATE:
            cur_transform.old_var = contacts[settings.selected_contact].angles;
            cur_transform.var = &contacts[settings.selected_contact].angles;
            break;
        case MoveMode::NONE:
        case MoveMode::SCALE:
            return;
        }
    }

    cur_transform.mode = cur_mode;
    cur_transform.start_mouse_pos = { last_mouse_x, last_mouse_y };
}

void Render_OnMouse(int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) right_mouse_down = true;
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) right_mouse_down = false;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) left_mouse_down = true;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) left_mouse_down = false;

    BeginEndTransform(button, action, mods);
    SelectBody(button, action, mods);
}

void Render_OnKey(int key, int action, int mods)
{
    if (key >= 0 && key < 512) {
        if (action == GLFW_PRESS) keys[key] = true;
        else if (action == GLFW_RELEASE) keys[key] = false;
    }

    if (action == GLFW_PRESS) {
        switch (key) {
        case GLFW_KEY_ESCAPE:
            settings.selected_body = -1;
            settings.selected_contact = -1;
            break;    
        case GLFW_KEY_DELETE:
            if (settings.selected_contact != -1) DeleteContact();
            else if (settings.selected_body != -1) DeleteBody();
            break;
        case GLFW_KEY_ENTER:
            settings.animate = !settings.animate;
            break;
        }   
    }
}

void Render_OnScroll(int xoffset, int yoffset)
{
    int idx = GetViewIdx();
    if (idx == -1) return;
    
    if (yoffset > 0) ortho_views[idx].scale /= 1.25f;
    if (yoffset < 0) ortho_views[idx].scale *= 1.25f;
}
