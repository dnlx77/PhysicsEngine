#pragma once
#include <memory>
#include "Physics/RigidBody.h"

class DistanceConstraint {
private:
	RigidBody *particleA;
	RigidBody *particleB;
	float restLength;
	float stiffness;

public:
	DistanceConstraint(RigidBody *a, RigidBody *b, float stiff = 1.0f);
	bool IsValid() const { return particleA && particleB; }
	RigidBody *GetParticleA() const { return particleA; }
	RigidBody *GetParticleB() const { return particleB; }
	void Solve();
};