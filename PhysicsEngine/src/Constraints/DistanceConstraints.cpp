#include "Constraints/DistanceConstraints.h"
#include "Constraints/Constraint.h"
#include <iostream>

DistanceConstraint::DistanceConstraint(RigidBody *a, RigidBody *b, float stiff) : Constraint(a, stiff), particleB(b)
{
	restLength = Vector2::Distance(a->position, b->position);
}

void DistanceConstraint::Solve()
{
	if (!IsValid()) return;

	Vector2 delta = particleB->position - particleA->position;
	float currentLength = delta.Length();
	float error = currentLength - restLength;

	//std::cout << "Error: " << error << " | Length: " << currentLength << " | Rest: " << restLength << std::endl;

	if (std::abs(error) < 1e-6f) return;

	Vector2 direction = delta / currentLength;

	float invMassTotal = particleA->inverseMass + particleB->inverseMass;

	if (invMassTotal < 1e-6f) return;

	error *= stiffness;

	Vector2 correction_A = direction * error * (particleA->inverseMass / invMassTotal);
	Vector2 correction_B = direction * error * (particleB->inverseMass / invMassTotal);

	particleA->position += correction_A;
	particleB->position -= correction_B;
}
