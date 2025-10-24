#pragma once
#include <memory>
#include "Constraints/Constraint.h"
#include "Physics/RigidBody.h"

class DistanceConstraint : public Constraint{
private:
	RigidBody *particleB;
	float restLength;

public:
	DistanceConstraint(RigidBody *a, RigidBody *b, float stiff = 1.0f);
	
	bool IsValid() const override { return particleA && particleB; }
	RigidBody *GetParticleA() const override { return particleA; }
	RigidBody *GetParticleB() const override { return particleB; }
	Vector2 GetPin() const override { return Vector2::ZERO; }
	void Solve() override;
};