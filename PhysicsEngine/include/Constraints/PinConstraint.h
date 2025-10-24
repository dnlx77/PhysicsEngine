#pragma once
#include "Constraints/Constraint.h"
#include "Physics/RigidBody.h"

class PinConstraint : public Constraint {
private:
    Vector2 pin;           // Punto fisso nello spazio
    float restLength;      // Distanza fissa dal pin al corpo

public:
    PinConstraint(RigidBody *b, Vector2 p, float stif);

    // TODO: Implementa i metodi virtuali
    void Solve() override;
    bool IsValid() const override { return (particleA!=nullptr); }
    RigidBody *GetParticleA() const override { return particleA; }
    RigidBody *GetParticleB() const override { return nullptr; }
    Vector2 GetPin() const override { return pin; }
};