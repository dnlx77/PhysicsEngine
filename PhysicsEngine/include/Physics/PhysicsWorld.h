#pragma once
#include "Physics/RigidBody.h"
#include "Collision/CollisionDetection.h"
#include <vector>
#include <memory>
#include <set>

class PhysicsWorld {
private:
    //int nextBodyId = 0;  // NUOVO: contatore ID
    std::vector<std::unique_ptr<RigidBody>> bodies;
    Vector2 gravity;
    float fixedTimeStep;        // Timestep fisso per stabilità
    float timeAccumulator;      // Accumula tempo per timestep fisso

    bool DetectCollision(RigidBody *a, RigidBody *b, CollisionInfo &info);
    void ApplyRestitution(const std::vector<CollisionInfo> &collisions);
    void ResolveCollision(const CollisionInfo &info);
    std::set<std::pair<RigidBody *, RigidBody *>> activeCollisions;
    void ClearPreviousCollisions();

public:
    PhysicsWorld();

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
    void SolvePositionConstraint(const CollisionInfo &info);
    void ApplyRestitution(const CollisionInfo &info);

    // Utility
    size_t GetBodyCount() const { return bodies.size(); }
    const std::vector<std::unique_ptr<RigidBody>> &GetBodies() const { return bodies; }
};