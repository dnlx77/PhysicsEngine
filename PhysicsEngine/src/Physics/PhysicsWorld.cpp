#include "Physics/PhysicsWorld.h"
#include "Collision/CollisionDetection.h"

PhysicsWorld::PhysicsWorld()
    : gravity(Vector2(0.0f, -9.8f)),
    fixedTimeStep(1.0f / 60.0f),
    timeAccumulator(0.0f),
    nextBodyId(0)
{
}

RigidBody *PhysicsWorld::CreateRigidBody(const Vector2 &position, float mass)
{
    bodies.emplace_back(std::make_unique<RigidBody>(position, mass));
    bodies.back()->id = nextBodyId++;
    return bodies.back().get();
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
    // Predizione: applica forze e aggiorna posizioni
    for (auto &body : bodies) {
        if (body->IsStatic() || !body->isActive)
            continue;

        body->previousPosition = body->position;
        Vector2 acceleration = gravity + (body->forceAccumulator * body->inverseMass);
        body->velocity += acceleration * fixedTimeStep;
        body->position += body->velocity * fixedTimeStep;
    }

    // Constraint solver: risolvi sovrapposizioni iterativamente
    std::vector<CollisionInfo> collisions;
    const int solverIterations = 5;

    for (int iteration = 0; iteration < solverIterations; iteration++) {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                CollisionInfo info;
                bool collided = DetectCollision(bodies[i].get(), bodies[j].get(), info);

                if (collided) {
                    if (iteration == 0) {
                        collisions.push_back(info);
                    }
                    SolvePositionConstraint(info);
                }
            }
        }
    }

    // Applica restituzione (rimbalzi)
    ApplyRestitution(collisions);

    // Aggiorna previousPosition per mantenere coerenza
    for (auto &body : bodies) {
        if (body->IsStatic() || !body->isActive)
            continue;
        body->previousPosition = body->position - body->velocity * fixedTimeStep;
    }

    // Pulisci forze accumulate
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
    return false;
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

    info.bodyA->position -= info.normal * correctionA;
    info.bodyB->position += info.normal * correctionB;
}

void PhysicsWorld::ApplyRestitution(const std::vector<CollisionInfo> &collisions)
{
    const float velocityThreshold = 0.01f;

    for (const auto &collision : collisions) {
        Vector2 relativeVelocity = collision.bodyB->velocity - collision.bodyA->velocity;
        float velocityAlongNormal = relativeVelocity.Dot(collision.normal);

        // Applica solo se si stanno avvicinando
        if (velocityAlongNormal < -velocityThreshold) {
            float e = std::min(collision.bodyA->restitution, collision.bodyB->restitution);
            float totalInvMass = collision.bodyA->inverseMass + collision.bodyB->inverseMass;
            float j = -(1.0f + e) * velocityAlongNormal / totalInvMass;
            Vector2 impulse = collision.normal * j;

            collision.bodyA->velocity -= impulse * collision.bodyA->inverseMass;
            collision.bodyB->velocity += impulse * collision.bodyB->inverseMass;
        }
    }
}