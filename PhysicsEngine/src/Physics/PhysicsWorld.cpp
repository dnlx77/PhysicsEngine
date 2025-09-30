#include "Physics/PhysicsWorld.h"
#include "Collision/CollisionDetection.h"
#include <algorithm>
#include <iostream>

void PhysicsWorld::ResolveCollision(const CollisionInfo &info)
{
	std::cout << "Collision! Penetration: " << info.penetration << " "
		<< " Normal: (" << info.normal.x << ", " << info.normal.y << ")" << std::endl;

	if (info.bodyA->IsStatic() && info.bodyB->IsStatic()) return;

	// Step 2: Risoluzione velocità
	// TODO: Calcola velocità relativa
	Vector2 relativeVelocity = info.bodyB->velocity - info.bodyA->velocity;

	// TODO: Velocità lungo la normale
	float velocityAlongNormal = relativeVelocity.Dot(info.normal);

	// Separazione con correction più forte
	const float percent = 0.8f;  // Percentage da correggere (80%)
	const float slop = 0.01f;    // Penetrazione permessa
	float correctionAmount = std::max(info.penetration - slop, 0.0f) * percent;

	//if (velocityAlongNormal > 0) return;
	
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

	// Applica l'impulso solo se si stanno avvicinando
	if (velocityAlongNormal < 0) {
		// TODO: Calcola impulso con restitution
		float e = std::min(info.bodyA->restitution, info.bodyB->restitution); // Minimo tra i due coefficienti di restitution
		float j = -1 * (1 + e) * velocityAlongNormal / (info.bodyA->inverseMass + info.bodyB->inverseMass); // Formula: -(1 + e) * velocityAlongNormal / (inverseMassA + inverseMassB)

		// TODO: Applica impulso
		Vector2 impulse = info.normal * j;
		info.bodyA->velocity -= impulse * info.bodyA->inverseMass;
		info.bodyB->velocity += impulse * info.bodyB->inverseMass;
	}
	/*
	// Dopo aver applicato l'impulso, controlla se mettere a dormire
	if (info.bodyB->IsStatic()) {
		// BodyA collide con statico
		if (info.bodyA->velocity.LengthSquared() < 0.05f && std::abs(velocityAlongNormal) < 0.1f) {
			info.bodyA->isSleeping = true;
			info.bodyA->velocity = Vector2::ZERO;
		}
	}
	if (info.bodyA->IsStatic()) {
		// BodyB collide con statico
		if (info.bodyB->velocity.LengthSquared() < 0.05f && std::abs(velocityAlongNormal) < 0.1f) {
			info.bodyB->isSleeping = true;
			info.bodyB->velocity = Vector2::ZERO;
		}
	}*/
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
		if (!body->IsStatic() && !body->isSleeping) {
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
			bool collided = false;

			if (bodies[i]->shapeType == ShapeType::CIRCLE &&
				bodies[j]->shapeType == ShapeType::CIRCLE) {
				collided = CollisionDetection::CircleVsCircle(bodies[i].get(), bodies[j].get(), info);
			}
			else if (bodies[i]->shapeType == ShapeType::AABB &&
				bodies[j]->shapeType == ShapeType::AABB) {
				collided = CollisionDetection::AABBvsAABB(bodies[i].get(), bodies[j].get(), info);
			}
			// CircleVsAABB lo faremo dopo

			if (collided) {
				ResolveCollision(info);
			}
		}
	}

	// Pulisce le forze
	for (auto &body : bodies) {
		body->ClearForces();
	}
}
