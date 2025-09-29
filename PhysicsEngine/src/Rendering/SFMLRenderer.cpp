#include "Rendering/SFMLRenderer.h"

SFMLRenderer::SFMLRenderer(unsigned int windowWidth,unsigned int windowHeight, float worldW, float worldH, const std::string &title) : worldWidth(worldW), worldHeight(worldH)
{ 
	window.create(sf::VideoMode({ windowWidth, windowHeight }), title);
}

bool SFMLRenderer::IsOpen() const
{
	return window.isOpen();
}

void SFMLRenderer::Clear()
{
	window.clear();
}

void SFMLRenderer::Display()
{
	window.display();
}

sf::Vector2f SFMLRenderer::WorldToScreen(const Vector2 &worldPos)
{
	float x = (window.getView().getSize().x / worldWidth) * worldPos.x;
	float y = -1 * ((window.getView().getSize().y / worldHeight) * worldPos.y) + window.getView().getSize().y - 1;
	return sf::Vector2f(x, y);
}

sf::CircleShape SFMLRenderer::CreateCircle(const RigidBody &body)
{
	float screenRadius = (window.getView().getSize().x / worldWidth) * body.radius;

	sf::CircleShape circle(screenRadius);
	circle.setFillColor(body.IsStatic() ? sf::Color::Color(128,128,128,255) : sf::Color::Blue);
	circle.setOrigin(sf::Vector2f(screenRadius, screenRadius));
	return circle;
}

void SFMLRenderer::HandleEvents()
{
	while (auto event = window.pollEvent()) {
		if (event->is<sf::Event::Closed>()) {
			window.close();
		}
	}
}

void SFMLRenderer::DrawWorld(const PhysicsWorld &world)
{
	const std::vector<std::unique_ptr<RigidBody>> &bodies = world.GetBodies();
	for (const auto &body : bodies) {
		sf::CircleShape circle = CreateCircle(*body);
		sf::Vector2f screenPos = WorldToScreen(body->position);
		circle.setPosition(screenPos);
		window.draw(circle);
	}
}
