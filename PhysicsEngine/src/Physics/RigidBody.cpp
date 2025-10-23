#include "Physics/RigidBody.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// Costruttori
RigidBody::RigidBody()
    : shapeType(ShapeType::CIRCLE), position(Vector2::ZERO), velocity(Vector2::ZERO), acceleration(Vector2::ZERO),
    angle(0.0f), angularVelocity(0.0f), angularAcceleration(0.0f),
    mass(1.0f), radius(1.0f), width(1.0f), height(1.0f), inverseMass(1.0f), inertia(1.0f), inverseInertia(1.0f),
    restitution(0.2f), friction(0.3f), isStatic(false), isActive(true), isSleeping(false),
    forceAccumulator(Vector2::ZERO), torqueAccumulator(0.0f)
{
    oldPosition = position;
}

RigidBody::RigidBody(Vector2 pos, float mass)
    : shapeType(ShapeType::CIRCLE), position(pos), velocity(Vector2::ZERO), acceleration(Vector2::ZERO),
    angle(0.0f), angularVelocity(0.0f), angularAcceleration(0.0f),
    radius(1.0f), width(1.0f), height(1.0f), restitution(0.2f), friction(0.3f), isStatic(false), isActive(true), isSleeping(false),
    forceAccumulator(Vector2::ZERO), torqueAccumulator(0.0f)
{
    SetMass(mass);
    UpdateInertia();
    oldPosition = position;
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

void RigidBody::UpdateInertia()
{
    if (isStatic) {
        inertia = 0.0f;
        inverseInertia = 0.0f;
        return;
    }

    if (shapeType == ShapeType::CIRCLE) {
        // Momento d'inerzia per un disco pieno: I = (1/2) * m * r²
        inertia = 0.5f * mass * radius * radius;
    }
    else if (shapeType == ShapeType::AABB) {
        // Momento d'inerzia per un rettangolo: I = (1/12) * m * (w² + h²)
        inertia = (mass / 12.0f) * (width * width + height * height);
    }

    // Calcola inverso (con safety check)
    inverseInertia = (inertia > 1e-6f) ? (1.0f / inertia) : 0.0f;
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

    UpdateInertia();
}

void RigidBody::SetRadius(float newRadius)
{
    radius = newRadius;
    if (shapeType == ShapeType::CIRCLE) {
        UpdateInertia();  // ✨ Ricalcola inerzia
    }
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

void RigidBody::Integrate(float dt)
{
    if (IsStatic()) return;

    // 🎯 VERLET INTEGRATION
    Vector2 acc = forceAccumulator * inverseMass;

    // Salva posizione corrente
    Vector2 temp = position;

    // Verlet: newPos = pos + (pos - oldPos) + acc * dt²
    position = position + (position - oldPosition) + acc * (dt * dt);

    // Update oldPosition
    oldPosition = temp;

    // Calcola velocità (per collision response e debug)
    if (dt > 1e-6f) {
        velocity = (position - oldPosition) / dt;
    }

    // integrazione angolare
    angularAcceleration = torqueAccumulator * inverseInertia;
    angularVelocity += angularAcceleration * dt;
    angle += angularVelocity * dt;
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
        oldPosition = position - velocity * (1.0f / 60.0f);
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
    UpdateInertia();
}

void RigidBody::UpdateVelocityFromPosition(const Vector2 &oldPos, float dt)
{
    if (dt > 1e-6f) {  // 🎯 Safety check aggiunto!
        velocity = (position - oldPos) / dt;
    }
}
