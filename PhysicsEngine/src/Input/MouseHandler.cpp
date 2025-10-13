#include "Input/MouseHandler.h"

RigidBody *MouseHandler::FindBodyAtPosition(const Vector2 &worldPos, PhysicsWorld &world)
{
    auto &bodies = world.GetBodies();
    for (auto &body : bodies) {
        if (body->shapeType == ShapeType::CIRCLE) {
            if (Vector2::DistanceSquared(body->position, worldPos) <= (body->radius * body->radius))
                return body.get();
        }
        else if (body->shapeType == ShapeType::AABB) {
            if (worldPos.x >= body->GetMinX() && worldPos.x <= body->GetMaxX() && worldPos.y >= body->GetMinY() && worldPos.y <= body->GetMaxY()) {
                return body.get();
            }
        }
    }
    return nullptr;
}

MouseHandler::MouseHandler(float stiffness, float damping)
    : dragStiffness(stiffness),
    dragDamping(damping),
    selectedBody(nullptr),
    mouseWorldPosition(Vector2::ZERO),
    clickOffsetLocal(Vector2::ZERO)
{
}

void MouseHandler::HandleMousePress(const Vector2 &worldPos, PhysicsWorld &world)
{
    mouseWorldPosition = worldPos;
    selectedBody = FindBodyAtPosition(mouseWorldPosition, world);

    if (selectedBody) {
        // Calcola offset in spazio locale (ruota inverso dell'angolo del corpo)
        Vector2 offsetWorld = worldPos - selectedBody->position;

        // Ruota l'offset di -angle per ottenere coordinate locali
        float cosA = std::cos(-selectedBody->angle);
        float sinA = std::sin(-selectedBody->angle);

        clickOffsetLocal.x = offsetWorld.x * cosA - offsetWorld.y * sinA;
        clickOffsetLocal.y = offsetWorld.x * sinA + offsetWorld.y * cosA;
    }
}

void MouseHandler::HandleMouseMove(const Vector2 &worldPos)
{
    mouseWorldPosition = worldPos;
}

void MouseHandler::HandleMouseRelease()
{
    selectedBody = nullptr;
    clickOffsetLocal = Vector2::ZERO;
}

void MouseHandler::Update(PhysicsWorld &world)
{
    if (!selectedBody) return;

    // Converti offset locale → mondo (ruota con l'angolo corrente)
    float cosA = std::cos(selectedBody->angle);
    float sinA = std::sin(selectedBody->angle);

    Vector2 clickOffsetWorld;
    clickOffsetWorld.x = clickOffsetLocal.x * cosA - clickOffsetLocal.y * sinA;
    clickOffsetWorld.y = clickOffsetLocal.x * sinA + clickOffsetLocal.y * cosA;

    // Ora usa clickOffsetWorld per i calcoli
    Vector2 attachPoint = selectedBody->position + clickOffsetWorld;
    Vector2 toMouse = mouseWorldPosition - attachPoint;
    Vector2 force = dragStiffness * toMouse;

    Vector2 damping = -dragDamping * selectedBody->velocity;
    force += damping;

    selectedBody->ApplyForce(force);

    float torque = -(clickOffsetWorld.x * force.y - clickOffsetWorld.y * force.x);
    float angularDamping = -dragDamping * 0.1f * selectedBody->angularVelocity;
    selectedBody->ApplyTorque(torque + angularDamping);
}
