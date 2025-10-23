#pragma once
#include "Math/Vector2.h"

class AABB {
public:
    Vector2 center;
    float halfWidth;
    float halfHeight;

    AABB(Vector2 center, float halfWidth, float halfHeight);

    bool Contains(const Vector2 &point) const;
    bool Intersects(const AABB &other) const;

    float GetMinX() const { return center.x - halfWidth; }
    float GetMaxX() const { return center.x + halfWidth; }
    float GetMinY() const { return center.y - halfHeight; }
    float GetMaxY() const { return center.y + halfHeight; }
};