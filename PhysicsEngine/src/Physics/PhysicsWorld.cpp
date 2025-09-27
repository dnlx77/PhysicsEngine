#include "Physics/PhysicsWorld.h"

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

	// Pulisce le forze
	for (auto &body : bodies) {
		body->ClearForces();
	}
}
