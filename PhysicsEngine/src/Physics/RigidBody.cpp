#include "Physics/RigidBody.h"
#include <cmath>
#include <algorithm>

// Costruttori
RigidBody::RigidBody()
    : shapeType(ShapeType::CIRCLE), position(Vector2::ZERO), velocity(Vector2::ZERO), acceleration(Vector2::ZERO),
    angle(0.0f), angularVelocity(0.0f), angularAcceleration(0.0f),
    mass(1.0f), radius(1.0f), width(1.0f), height(1.0f), inverseMass(1.0f), inertia(1.0f), inverseInertia(1.0f),
    restitution(0.2f), friction(0.3f), isStatic(false), isActive(true), isSleeping(false),
    forceAccumulator(Vector2::ZERO), torqueAccumulator(0.0f)
{
}

RigidBody::RigidBody(Vector2 pos, float mass)
    : shapeType(ShapeType::CIRCLE), position(pos), velocity(Vector2::ZERO), acceleration(Vector2::ZERO),
    angle(0.0f), angularVelocity(0.0f), angularAcceleration(0.0f),
    radius(1.0f), width(1.0f), height(1.0f), restitution(0.2f), friction(0.3f), isStatic(false), isActive(true), isSleeping(false),
    forceAccumulator(Vector2::ZERO), torqueAccumulator(0.0f)
{
    SetMass(mass);
    SetInertia(mass);  // Inerzia semplificata per iniziare
}

void RigidBody::ApplyForce(const Vector2 &force)
{
    if (!isStatic) {
        isSleeping = false;  // Risveglia quando riceve forze
        forceAccumulator += force;
    }
}

void RigidBody::ApplyTorque(float torque)
{
    if (!isStatic) {
        torqueAccumulator += torque;
    }
}

void RigidBody::SetMass(float newMass)
{
    if (newMass <= 0.0f) {
        // Massa infinita = oggetto statico
        mass = 0.0f;
        inverseMass = 0.0f;
        isStatic = true;
    }
    else {
        mass = newMass;
        inverseMass = 1.0f / mass;
        // Non modifichiamo isStatic qui - può essere impostato separatamente
    }
}

void RigidBody::SetRadius(float newRadius)
{
    radius = newRadius;
}

void RigidBody::SetInertia(float newInertia)
{
    if (newInertia <= 0.0f) {
        inertia = 0.0f;
        inverseInertia = 0.0f;
    }
    else {
        inertia = newInertia;
        inverseInertia = 1.0f / inertia;
    }
}

void RigidBody::SetStatic(bool static_state)
{
    isStatic = static_state;
    if (isStatic) {
        velocity = Vector2::ZERO;
        angularVelocity = 0.0f;
    }
}

void RigidBody::Integrate(float deltaTime)
{
    if (isStatic || !isActive)
        return;

    // Traslazione
    acceleration = forceAccumulator * inverseMass;
    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;

    // Rotazione
    angularAcceleration = torqueAccumulator * inverseInertia;
    angularVelocity += angularAcceleration * deltaTime;
    angle += angularVelocity * deltaTime;
}

void RigidBody::ClearForces()
{
    forceAccumulator = Vector2::ZERO;
    torqueAccumulator = 0.0f;
}

void RigidBody::SetPosition(const Vector2 &pos)
{
    position = pos;
}

void RigidBody::SetVelocity(const Vector2 &vel)
{
    if (!isStatic) {
        velocity = vel;
    }
}

void RigidBody::SetAngle(float newAngle)
{
    angle = newAngle;
}

void RigidBody::SetAngularVelocity(float newAngularVel)
{
    if (!isStatic) {
        angularVelocity = newAngularVel;
    }

}

void RigidBody::SetAABB(float w, float h)
{
    shapeType = ShapeType::AABB;
    width = w;
    height = h;
}
