#include "physics.h"

#define SIZE_X 3
#define SIZE_Y 3

int main()
{
    bodies.push_back(Body {
        INFINITY, {0, 0, 0}, {0, -1, 0}, {1000, 1, 1000}, {0, 0, 0}
    });

    for (int i = 0; i < SIZE_Y; i++) {
        for (int j = 0; j < SIZE_X; j++) {
            bodies.push_back(Body {
                1, {5, -9.8, 3}, {(double)j, (double)i, 0}, {1, 1, 1}, {0, 0, 0}
            });
        }
    }

    for (int i = 0; i < SIZE_X; i++) {
        Contact c;
        c.i = 0;
        c.j = i + 1;
        c.angles = {0, 0, 0};
        c.mu = 0.3;

        c.pos = {(double)i + 0.5, -0.5, 0.5};
        contacts.push_back(c);

        c.pos = {(double)i + 0.5, -0.5, -0.5};
        contacts.push_back(c);

        c.pos = {(double)i - 0.5, -0.5, 0.5};
        contacts.push_back(c);

        c.pos = {(double)i - 0.5, -0.5, -0.5};
        contacts.push_back(c);
    }

    int idx = SIZE_X+1;
    for (int i = 1; i < SIZE_Y; i++) {
        for (int j = 0; j < SIZE_X; j++) {
            Contact c;
            c.i = idx - SIZE_X;
            c.j = idx;
            c.angles = {0, 0, 0};
            c.mu = 0.3;
            
            c.pos = {(double)j + 0.5, -0.5 + i, 0.5};
            contacts.push_back(c);
            
            c.pos = {(double)j + 0.5, -0.5 + i, -0.5};
            contacts.push_back(c);

            c.pos = {(double)j - 0.5, -0.5 + i, 0.5};
            contacts.push_back(c);

            c.pos = {(double)j - 0.5, -0.5 + i, -0.5};
            contacts.push_back(c);
            
            idx++;
        }
    }

    idx = 1;
    for (int i = 0; i < SIZE_Y; i++) {
        for (int j = 1; j < SIZE_X; j++) {
            Contact c;
            c.i = idx;
            c.j = idx + 1;
            c.angles = {0, 0, -90.0};
            c.mu = 0.3;
            
            c.pos = {(double)j - 0.5, (double)i + 0.5, 0.5};
            contacts.push_back(c);
            
            c.pos = {(double)j - 0.5, (double)i + 0.5, -0.5};
            contacts.push_back(c);
            
            c.pos = {(double)j - 0.5, (double)i - 0.5, 0.5};
            contacts.push_back(c);
            
            c.pos = {(double)j - 0.5, (double)i - 0.5, -0.5};
            contacts.push_back(c);
            

            idx++;
        }
        idx++;
    }

    Phys_SaveToFile("examples/brick_wall.bin");
}