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

void SFMLRenderer::DrawLine(const Vector2 &start, const Vector2 &end, sf::Color color)
{
    sf::Vector2f screenStart = WorldToScreen(start);
    sf::Vector2f screenEnd = WorldToScreen(end);

    sf::Vertex line[] = {
        sf::Vertex(screenStart, color),
        sf::Vertex(screenEnd, color)
    };

    window.draw(line, 2, sf::PrimitiveType::Lines);
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

Vector2 SFMLRenderer::ScreenToWorld(const sf::Vector2i screenPos)
{
    float x = (worldWidth / window.getView().getSize().x) * screenPos.x;
    float y = (window.getView().getSize().y - 1 - screenPos.y) * (worldHeight / window.getView().getSize().y);
    return Vector2(x, y);
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
    const auto &bodies = world.GetBodies();

    // Disegna bodies
    for (const auto &body : bodies) {
        sf::Vector2f screenPos = WorldToScreen(body->position);

        if (body->shapeType == ShapeType::CIRCLE) {
            // Crea cerchio
            float screenRadius = (window.getView().getSize().x / worldWidth) * body->radius;
            sf::CircleShape circle(screenRadius);
            circle.setOrigin(sf::Vector2f(screenRadius, screenRadius));
            circle.setFillColor(body->IsStatic() ? sf::Color::Color(128, 128, 128, 255) : sf::Color::Blue);
            circle.setPosition(screenPos);
            window.draw(circle);

            // ✨ Linea rossa per vedere la rotazione
            sf::RectangleShape indicator(sf::Vector2f(screenRadius, 3));
            indicator.setOrigin(sf::Vector2f(0, 1.5f));
            indicator.setPosition(screenPos);
            indicator.setRotation(sf::radians(body->angle));
            indicator.setFillColor(sf::Color::Red);
            window.draw(indicator);
        }
        else if (body->shapeType == ShapeType::AABB) {
            // Crea rettangolo
            float screenWidth = (window.getView().getSize().x / worldWidth) * body->width;
            float screenHeight = (window.getView().getSize().y / worldHeight) * body->height;
            sf::RectangleShape rect(sf::Vector2f(screenWidth, screenHeight));
            rect.setOrigin(sf::Vector2f(screenWidth / 2, screenHeight / 2));
            rect.setFillColor(body->IsStatic() ? sf::Color::Color(128, 128, 128, 255) : sf::Color::Green);
            rect.setPosition(screenPos);
            rect.setRotation(sf::radians(body->angle));
            window.draw(rect);
        }
    }

    // Disegna constraints
    const auto &constraints = world.GetConstraints();
    for (const auto &c : constraints) {
        if (c->IsValid()) {
            DrawLine(
                c->GetParticleA()->position,
                c->GetParticleB()->position,
                sf::Color(100, 100, 100)
            );
        }
    }
}
