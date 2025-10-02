#include "Collision/AABB.h"

AABB::AABB(Vector2 center, float halfWidth, float halfHeight)
    : center(center), halfWidth(halfWidth), halfHeight(halfHeight)
{
}

bool AABB::Contains(const Vector2 &point) const
{
    // TODO: Verifica se point è dentro i bounds
    // point.x tra [minX, maxX] E point.y tra [minY, maxY]

    if (point.x >= GetMinX() && point.x <= GetMaxX() && point.y >= GetMinY() && point.y <= GetMaxY())
        return true;

    return false;
}

bool AABB::Intersects(const AABB &other) const
{
    // TODO: Usa la stessa logica di AABBvsAABB collision
    // Se si sovrappongono su entrambi gli assi -> true

    float XOverlap = std::fmin(GetMaxX() - other.GetMinX(), other.GetMaxX() - GetMinX());
    float YOverlap = std::fmin(GetMaxY() - other.GetMinY(), other.GetMaxY() - GetMinY());

    if (XOverlap > 0 && YOverlap > 0)
        return true;

    return false;
}