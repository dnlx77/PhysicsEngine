#include "Collision/CollisionDetection.h"
#include <algorithm>

bool CollisionDetection::CircleVsCircle(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
	float distance = Vector2::Distance(a->position, b->position);
	float penetration = (a->radius + b->radius) - distance;

	const float collisionThreshold = 0.001f;  // NUOVO: ignora micro-sovrapposizioni

	if (penetration > collisionThreshold) {  // Cambiato da > 0
		info.bodyA = a;
		info.bodyB = b;
		info.normal = (b->position - a->position).Normalized();
		info.penetration = penetration;
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

bool CollisionDetection::CircleVsAABB(RigidBody *circle, RigidBody *aabb, CollisionInfo &info)
{
	info.bodyA = circle;
	info.bodyB = aabb;
	float XOverlap = std::clamp(circle->position.x, aabb->GetMinX(), aabb->GetMaxX());
	float YOverlap = std::clamp(circle->position.y, aabb->GetMinY(), aabb->GetMaxY());
	Vector2 point(XOverlap, YOverlap);
	float distance = Vector2::Distance(circle->position, point);

	if (distance < circle->radius) {
		if (distance < 1e-6f) {
			info.normal = Vector2::UP;
		}
		else {
			info.normal = (point - circle->position).Normalized();
		}

		info.penetration = circle->radius - distance;
		info.hasCollision = true;
		return true;
	}

	info.normal = Vector2::ZERO;
	info.penetration = 0.0f;
	info.hasCollision = false;
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
