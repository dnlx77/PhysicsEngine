#include "Collision/CollisionDetection.h"

bool CollisionDetection::CircleVsCircle(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
	float distance = Vector2::Distance(a->position, b->position);
	if (distance < (a->radius + b->radius)) {
		info.bodyA = a;
		info.bodyB = b;
		info.normal = (b->position - a->position).Normalized();
		info.penetration = (a->radius + b->radius) - distance;
		info.hasCollision = true;
		return true;
	}

	info.bodyA = a;
	info.bodyB = b;
	info.normal = Vector2::ZERO;
	info.penetration = 0.0f;
	info.hasCollision = false;
	return false;
}

bool CollisionDetection::CircleVsGround(RigidBody *circle, float groundY, CollisionInfo &info)
{
	if (circle->position.y - circle->radius <= groundY) {
		info.normal = Vector2(0, 1);  // Sempre verso l'alto
		info.penetration = groundY - (circle->position.y - circle->radius);
		return true;
	}
	return false;
}
