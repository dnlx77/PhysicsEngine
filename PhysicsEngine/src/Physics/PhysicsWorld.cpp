#include "Physics/PhysicsWorld.h"
#include "Collision/CollisionDetection.h"
#include <algorithm>
#include <iostream>

void PhysicsWorld::ResolveCollision(const CollisionInfo &info)
{
	std::cout << "Collision! Penetration: " << info.penetration
		<< " Normal: (" << info.normal.x << ", " << info.normal.y << ")" << std::endl;

	if (info.bodyA->IsStatic() && info.bodyB->IsStatic()) return;
	
	if (info.bodyA->IsStatic()) {
		// BodyB è dinamico
		info.bodyB->position -= info.normal * info.penetration;
	}
	else if (info.bodyB->IsStatic()) {
		// BodyA è dinamico
		info.bodyA->position += info.normal * info.penetration;
	}
	else {
		// Entrambi dinamici
		info.bodyA->position -= (info.normal * info.penetration) / 2;
		info.bodyB->position += (info.normal * info.penetration) / 2;
	}

	// Step 2: Risoluzione velocità
	// TODO: Calcola velocità relativa
	Vector2 relativeVelocity = info.bodyB->velocity - info.bodyA->velocity;

	// TODO: Velocità lungo la normale
	float velocityAlongNormal = relativeVelocity.Dot(info.normal);

	// TODO: Se si stanno già allontanando, non fare niente
	if (velocityAlongNormal > 0) return;

	// TODO: Calcola impulso con restitution
	float e = std::min(info.bodyA->restitution, info.bodyB->restitution); // Minimo tra i due coefficienti di restitution
	float j = -1 * (1 + e) * velocityAlongNormal / (info.bodyA->inverseMass + info.bodyB->inverseMass); // Formula: -(1 + e) * velocityAlongNormal / (inverseMassA + inverseMassB)

	// TODO: Applica impulso
	Vector2 impulse = info.normal * j;
	info.bodyA->velocity -= impulse * info.bodyA->inverseMass;
	info.bodyB->velocity += impulse * info.bodyB->inverseMass;
}

PhysicsWorld::PhysicsWorld() : gravity(Vector2(0.0f,-9.8f)), fixedTimeStep(1.0f / 60.0f), timeAccumulator(0.0f)
{
}

RigidBody *PhysicsWorld::CreateRigidBody(const Vector2 &position, float mass)
{
	bodies.emplace_back(std::make_unique<RigidBody>(position, mass));
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
	// Applica la gravita a tutti i corpi
	for (auto &body : bodies) {
		if (!body->IsStatic()) {
			body->ApplyForce(gravity * body->GetMass());
		}
	}

	// Integra tutti i corpi
	for (auto &body : bodies) {
		body->Integrate(fixedTimeStep);
	}

	// Collision detection
	for (size_t i = 0; i < bodies.size(); ++i) {
		for (size_t j = i + 1; j < bodies.size(); ++j) {
			CollisionInfo info;
			if (CollisionDetection::CircleVsCircle(bodies[i].get(), bodies[j].get(), info)) {
				// TODO: Risolvi la collisione
				ResolveCollision(info);
			}
		}
	}

	// Pulisce le forze
	for (auto &body : bodies) {
		body->ClearForces();
	}
}
