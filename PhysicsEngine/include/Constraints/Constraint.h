#pragma once
#include "Math/Vector2.h"

class RigidBody;  // Forward declaration

class Constraint {
protected:
    RigidBody *particleA;
    float stiffness;

    Constraint(RigidBody* a, float stiff = 1.0f);
public:
    virtual ~Constraint();

    // Metodi virtuali puri - ogni vincolo li implementa
    virtual void Solve() = 0;
    virtual bool IsValid() const = 0;
    virtual RigidBody *GetParticleA() const = 0;
    virtual RigidBody *GetParticleB() const = 0;  // Ritornerà nullptr per Pin
    virtual Vector2 GetPin() const = 0;           // Ritornerà ZERO se non è un Pin
};