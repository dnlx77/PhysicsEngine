#include "Constraints/Constraint.h"

Constraint::Constraint(RigidBody *a, float stiff) : particleA(a), stiffness(stiff)
{
}

Constraint::~Constraint() = default;

