#pragma once
#include "Physics/PhysicsWorld.h"
#include <vector>

class ConsoleRenderer {
private:
    int width, height;
    float worldWidth, worldHeight;
    std::vector<std::vector<char>> buffer;

public:
    ConsoleRenderer(int w, int h, float worldW, float worldH);

    void Clear();
    void DrawWorld(const PhysicsWorld &world);
    void Present();

private:
    void WorldToScreen(const Vector2 &worldPos, int &screenX, int &screenY);
    void SetPixel(int x, int y, char c);
};