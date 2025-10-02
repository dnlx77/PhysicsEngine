#pragma once
#include "Physics/RigidBody.h"
#include "Collision/CollisionDetection.h"
#include "Collision/QuadTree.h"
#include <vector>
#include <memory>

class PhysicsWorld {
private:
    std::vector<std::unique_ptr<RigidBody>> bodies;
    Vector2 gravity;
    float fixedTimeStep;        // Timestep fisso per stabilità
    float timeAccumulator;      // Accumula tempo per timestep fisso

    void ResolveCollision(const CollisionInfo &info);
    AABB worldBounds;


public:
    PhysicsWorld();
    PhysicsWorld(const AABB &bounds);

    // Gestione RigidBody
    RigidBody *CreateRigidBody(const Vector2 &position, float mass);
    void RemoveRigidBody(RigidBody *body);
    void Clear();

    // Impostazioni mondo fisico
    void SetGravity(const Vector2 &g);
    void SetTimeStep(float timeStep);
    Vector2 GetGravity() const { return gravity; }

    // Simulazione
    void Update(float deltaTime);    // Aggiorna con timestep variabile
    void Step();                     // Un singolo step di simulazione

    // Utility
    size_t GetBodyCount() const { return bodies.size(); }
    const std::vector<std::unique_ptr<RigidBody>> &GetBodies() const { return bodies; }
};