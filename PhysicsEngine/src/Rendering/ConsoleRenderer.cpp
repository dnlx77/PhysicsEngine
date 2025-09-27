#include "Rendering/ConsoleRenderer.h"
#include <cstdlib>
#include <iostream>

ConsoleRenderer::ConsoleRenderer(int w, int h, float worldW, float worldH) : width(w), height(h), worldWidth(worldW), worldHeight(worldH)
{
	buffer = std::vector<std::vector<char>>(height, std::vector<char>(width, ' '));
}

void ConsoleRenderer::Clear()
{
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; j++)
			buffer[i][j] = ' ';
}

void ConsoleRenderer::DrawWorld(const PhysicsWorld &world)
{
	int x, y;
	const std::vector<std::unique_ptr<RigidBody>> &bodies = world.GetBodies();

	for (auto &body : bodies)
	{
		WorldToScreen(body->position, x, y);

		if (body->IsStatic())
			SetPixel(x, y, '#');
		else
			SetPixel(x, y, 'O');
	}
}

void ConsoleRenderer::Present()
{
	system("cls");

	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; j++) {
			std::cout<<buffer[i][j];
		}
		std::cout << std::endl;
	}
}

void ConsoleRenderer::WorldToScreen(const Vector2 &worldPos, int &screenX, int &screenY)
{
	screenX = (int)((width / worldWidth) * worldPos.x);
	screenY = (int)(- 1 * (height / worldHeight) * worldPos.y);
	screenY += height - 1;
}

void ConsoleRenderer::SetPixel(int x, int y, char c)
{
	if ((x < width) && (y < height)) {
		buffer[y][x] = c;
	}
	else
		throw std::out_of_range("Point is out of range!");
}
