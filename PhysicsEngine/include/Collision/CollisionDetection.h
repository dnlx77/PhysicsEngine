#pragma once
#include "Physics/RigidBody.h"

struct CollisionInfo {
    RigidBody *bodyA;
    RigidBody *bodyB;
    Vector2 normal;        // Direzione della collisione (da A verso B)
    float penetration;     // Quanto si sovrappongono
    bool hasCollision;     // Se c'è davvero collisione
};

class CollisionDetection {
public:
    static bool CircleVsCircle(RigidBody *a, RigidBody *b, CollisionInfo &info);
    static bool CircleVsGround(RigidBody *circle, float groundY, CollisionInfo &info);
};