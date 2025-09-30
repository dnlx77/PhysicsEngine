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

bool CollisionDetection::AABBvsAABB(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
	info.bodyA = a;
	info.bodyB = b;
	info.normal = Vector2::ZERO;
	info.penetration = 0.0f;
	info.hasCollision = false;

	float XOverlap = std::fmin(a->GetMaxX() - b->GetMinX(), b->GetMaxX() - a->GetMinX());
	float YOverlap = std::fmin(a->GetMaxY() - b->GetMinY(), b->GetMaxY() - a->GetMinY());

	if (XOverlap <= 0 || YOverlap <= 0) return false;

	if (XOverlap < YOverlap) {
		info.normal = (a->position.x < b->position.x ? Vector2::RIGHT : Vector2::LEFT);
		info.penetration = XOverlap;
	}
	else {
		info.normal = (a->position.y < b->position.y ? Vector2::UP : Vector2::DOWN);
		info.penetration = YOverlap;
	}

	info.hasCollision = true;
	return true;
}
