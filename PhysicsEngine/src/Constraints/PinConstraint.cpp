#include "Constraints/Constraint.h"
#include "Constraints/PinConstraint.h"

PinConstraint::PinConstraint(RigidBody *a, Vector2 p, float stif) : Constraint(a, stif), pin(p)
{
	restLength = Vector2::Distance(a->position, p);
}

void PinConstraint::Solve()
{
	if (!IsValid()) return;

	Vector2 delta = pin - particleA->position;
	float currentLength = delta.Length();
	float error = currentLength - restLength;

	//std::cout << "Error: " << error << " | Length: " << currentLength << " | Rest: " << restLength << std::endl;

	if (std::abs(error) < 1e-6f) return;

	if (std::abs(currentLength) < 1e-6f) return;

	Vector2 direction = delta / currentLength;

	error *= stiffness;

	Vector2 correction_A = direction * error * particleA->inverseMass;

	particleA->position += correction_A;
}
