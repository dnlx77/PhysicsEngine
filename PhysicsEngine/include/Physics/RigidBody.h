#pragma once
#include "Math/Vector2.h"

class RigidBody {
public:
    // Proprietà cinematiche
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;

    // Proprietà rotazionali
    float angle;                   // Angolo in radianti
    float angularVelocity;         // Velocità angolare
    float angularAcceleration;     // Accelerazione angolare

    // Proprietà fisiche
    float mass;
    float radius;                  // Raggio del corpo
    float inverseMass;             // 1/mass per ottimizzazione
    float inertia;                 // Momento di inerzia
    float inverseInertia;          // 1/inertia per ottimizzazione
    float restitution;             // Coefficiente di restituzione (0-1)
    float friction;                // Coefficiente di attrito (0-1)

    // Stato dell'oggetto
    bool isStatic;                 // true = massa infinita, non si muove
    bool isActive;                 // true = partecipa alla simulazione
    bool isSleeping;               // true = non subisce la gravità

private:
    // Accumulo delle forze
    Vector2 forceAccumulator;      // Somma di tutte le forze applicate
    float torqueAccumulator;       // Somma di tutti i momenti applicati

public:
    // Costruttori
    RigidBody();
    RigidBody(Vector2 pos, float mass);

    // Metodi per applicare forze
    void ApplyForce(const Vector2 &force);
    void ApplyForceAtPoint(const Vector2 &force, const Vector2 &point);
    void ApplyTorque(float torque);
    void ApplyImpulse(const Vector2 &impulse);
    void ApplyImpulseAtPoint(const Vector2 &impulse, const Vector2 &point);

    // Metodi per impostare proprietà
    void SetMass(float newMass);
    void SetRadius(float newRadius);
    void SetInertia(float newInertia);
    void SetStatic(bool static_state);

    // Metodi di simulazione
    void Integrate(float deltaTime);    // Integrazione di Eulero
    void ClearForces();                 // Pulisce accumulatori forze

    // Utility
    Vector2 GetPointVelocity(const Vector2 &point) const;  // Velocità di un punto del corpo
    void SetPosition(const Vector2 &pos);
    void SetVelocity(const Vector2 &vel);
    void SetAngle(float newAngle);
    void SetAngularVelocity(float newAngularVel);

    // Getters
    bool IsStatic() const { return isStatic; }
    bool IsActive() const { return isActive; }
    bool IsSleeping() const { return isSleeping; }
    float GetMass() const { return mass; }
    float getRadius() const { return radius; }
    float GetInverseMass() const { return inverseMass; }
};