#pragma once
#include "Physics/PhysicsWorld.h"
#include <SFML/Graphics.hpp>

class SFMLRenderer {
private:
    sf::RenderWindow window;
    float worldWidth, worldHeight;

public:
    SFMLRenderer(unsigned int windowWidth, unsigned int windowHeight, float worldW, float worldH, const std::string &title);

    bool IsOpen() const;
    void HandleEvents();
    void Clear();
    void DrawLine(const Vector2 &start, const Vector2 &end, sf::Color color = sf::Color::White);
    void DrawWorld(const PhysicsWorld &world);
    void HighlightBody(RigidBody* body);
    void DrawDragLine(Vector2 from, Vector2 to);
    void DrawDebugInfo(int bodyCount);
    Vector2 ScreenToWorld(const sf::Vector2i screenPos);
    void Display();
    sf::RenderWindow &GetWindow() { return window; }

private:
    sf::Vector2f WorldToScreen(const Vector2 &worldPos);
    sf::Font font;
    std::optional<sf::Text> debugText;
};