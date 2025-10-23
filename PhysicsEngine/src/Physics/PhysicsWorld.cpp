#include "Physics/PhysicsWorld.h"
#include "Collision/CollisionDetection.h"
#include <iostream>

PhysicsWorld::PhysicsWorld()
    : gravity(Vector2(0.0f, -9.8f)),
    fixedTimeStep(1.0f / 60.0f),
    timeAccumulator(0.0f)
{
    // Crea QuadTree per tutto il mondo (es. 20x15 centrato in 10, 7.5)
    AABB worldBounds(Vector2(10, 7.5f), 10.0f, 7.5f);
    quadTree = std::make_unique<QuadTree>(worldBounds, 4);  // capacity = 4
}

RigidBody *PhysicsWorld::CreateRigidBody(const Vector2 &position, float mass)
{
    bodies.emplace_back(std::make_unique<RigidBody>(position, mass));
    return bodies.back().get();
}

DistanceConstraint *PhysicsWorld::CreateDistanceConstraint(RigidBody *bodyA, RigidBody *bodyB, float stif)
{
    constraints.emplace_back(std::make_unique<DistanceConstraint>(bodyA, bodyB, stif));
    return constraints.back().get();
}

void PhysicsWorld::SetGravity(const Vector2 &g)
{
    gravity = g;
}

void PhysicsWorld::SetTimeStep(float timeStep)
{
    fixedTimeStep = timeStep;
}

void PhysicsWorld::Update(float deltaTime)
{
    timeAccumulator += deltaTime;
    while (timeAccumulator >= fixedTimeStep) {
        Step();
        timeAccumulator -= fixedTimeStep;
    }
}

void PhysicsWorld::Step()
{
    // 1. Applica gravità a tutti i corpi dinamici
    for (auto &body : bodies) {
        if (!body->IsStatic() && body->isActive) {
            body->ApplyForce(gravity * body->mass);
        }
    }

    // 2. Integra movimento (Verlet)
    for (auto &body : bodies) {
        if (body->IsStatic() || !body->isActive)
            continue;

        body->Integrate(fixedTimeStep);
    }

    // 3. Risolvi collisioni (position constraints)
    std::vector<CollisionInfo> collisions;
    const int solverIterations = 5;
    
    for (int iteration = 0; iteration < solverIterations; iteration++) {
        if (iteration == 0) {
            quadTree->Clear();
            for (auto &body : bodies) {
                quadTree->Insert(body.get());
            }
        }

        // Usa QuadTree invece del doppio loop
        for (auto &body : bodies) {
            std::vector<RigidBody *> nearby;

            // Crea AABB di query (espandi un po' per sicurezza)
            float querySize = body->shapeType == ShapeType::CIRCLE ?
                body->radius * 3.0f :
                std::max(body->width, body->height) * 2.0f;
            AABB queryRange(body->position, querySize, querySize);

            quadTree->Query(queryRange, nearby);

            for (auto *other : nearby) {
                if (body.get() >= other) continue;  // Evita duplicati e self-check

                CollisionInfo info;
                bool collided = DetectCollision(body.get(), other, info);
                if (collided) {
                    if (iteration == 0) {
                        collisions.push_back(info);
                    }
                    SolvePositionConstraint(info);
                }
            }
        }

        // Constraints
        for (auto &constraint : constraints) {
            constraint->Solve();
        }
    }

    // 4. Applica restituzione (rimbalzi)
    ApplyRestitution(collisions);

    // 5. Pulisci forze accumulate
    for (auto &body : bodies) {
        body->ClearForces();
    }
}

bool PhysicsWorld::DetectCollision(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
    if (a->shapeType == ShapeType::CIRCLE && b->shapeType == ShapeType::CIRCLE) {
        return CollisionDetection::CircleVsCircle(a, b, info);
    }
    else if (a->shapeType == ShapeType::AABB && b->shapeType == ShapeType::AABB) {
        return CollisionDetection::AABBvsAABB(a, b, info);
    }
    else {
        if (a->shapeType == ShapeType::CIRCLE)
            return CollisionDetection::CircleVsAABB(a, b, info);
        else
            return CollisionDetection::CircleVsAABB(b, a, info);
    }
}

void PhysicsWorld::SolvePositionConstraint(const CollisionInfo &info)
{
    if (info.bodyA->isStatic && info.bodyB->isStatic)
        return;

    float totalInverseMass = info.bodyA->inverseMass + info.bodyB->inverseMass;
    if (totalInverseMass == 0.0f)
        return;

    float correctionA = (info.bodyA->inverseMass / totalInverseMass) * info.penetration;
    float correctionB = (info.bodyB->inverseMass / totalInverseMass) * info.penetration;

    Vector2 correctionVecA = info.normal * correctionA;
    Vector2 correctionVecB = info.normal * correctionB;

    // 🎯 Aggiorna ENTRAMBI position E oldPosition
    if (!info.bodyA->isStatic) {
        info.bodyA->position -= correctionVecA;
        info.bodyA->oldPosition -= correctionVecA;  // ✅ Mantiene velocità!
    }

    if (!info.bodyB->isStatic) {
        info.bodyB->position += correctionVecB;
        info.bodyB->oldPosition += correctionVecB;  // ✅ Mantiene velocità!
    }
}

void PhysicsWorld::ApplyRestitution(const std::vector<CollisionInfo> &collisions)
{
    for (const auto &info : collisions) {
        RigidBody *bodyA = info.bodyA;
        RigidBody *bodyB = info.bodyB;

        Vector2 velA = (bodyA->position - bodyA->oldPosition) / fixedTimeStep;
        Vector2 velB = (bodyB->position - bodyB->oldPosition) / fixedTimeStep;

        Vector2 relativeVelocity = velB - velA;
        float velocityAlongNormal = relativeVelocity.Dot(info.normal);

        if (velocityAlongNormal > 0)
            continue;

        float restitution = std::min(bodyA->restitution, bodyB->restitution);
        float j = -(1.0f + restitution) * velocityAlongNormal;
        j /= (bodyA->inverseMass + bodyB->inverseMass);

        Vector2 impulse = info.normal * j;

        // 🎯 CORREZIONE DEFINITIVA: Segni ORIGINALI erano giusti!
        if (!bodyA->IsStatic()) {
            bodyA->oldPosition += impulse * bodyA->inverseMass * fixedTimeStep;  // ✅
        }

        if (!bodyB->IsStatic()) {
            bodyB->oldPosition -= impulse * bodyB->inverseMass * fixedTimeStep;  // ✅
        }
    }
}