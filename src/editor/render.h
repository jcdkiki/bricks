#ifndef BRICKS_DRAW_H
#define BRICKS_DRAW_H

void Render_Draw();
void Render_OnMouse(int button, int action, int mods);
void Render_OnKey(int key, int action, int mods);
void Render_UpdateCamera(double dt);
void Render_OnScroll(int xoffset, int yoffset);

#endif
