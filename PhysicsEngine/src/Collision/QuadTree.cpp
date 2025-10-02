#include "Collision/QuadTree.h"

QuadTree::QuadTree(const AABB &boundary, int capacity)
    : boundary(boundary), capacity(capacity), divided(false)
{
}

void QuadTree::Clear() {
    objects.clear();

    if (divided) {
        northWest.reset();
        northEast.reset();
        southWest.reset();
        southEast.reset();
        divided = false;
    }
}

bool QuadTree::Insert(RigidBody *body)
{
    if (!boundary.Contains(body->position))
        return false;
    
    if (objects.size() < capacity && !divided) {
        objects.push_back(body);
        return true;
    }

    if (objects.size() >= capacity && !divided) {
        Subdivide();

        // Redistribuisci oggetti esistenti nei figli
        for (auto *obj : objects) {
            northWest->Insert(obj) || northEast->Insert(obj) ||
            southWest->Insert(obj) || southEast->Insert(obj);
        }

        // Svuota questo nodo (ora gli oggetti sono nei figli)
        objects.clear();
    }

    if (northWest->Insert(body) || northEast->Insert(body) || 
        southWest->Insert(body) || southEast->Insert(body))
        return true;

    return false;
}

void QuadTree::Query(const AABB &range, std::vector<RigidBody *> &found)
{
    if (!boundary.Intersects(range))
        return;

    for (auto obj : objects) {
        if (range.Contains(obj->position))
            found.push_back(obj);
    }

    if (divided) {
        northWest->Query(range, found);
        northEast->Query(range, found);
        southWest->Query(range, found);
        southEast->Query(range, found);
    }
}

void QuadTree::Subdivide()
{
    // TODO: Crea 4 figli dividendo l'area in 4 quadranti
    // NorthWest: centro in (x - halfW/2, y + halfH/2), dimensioni halfW/2, halfH/2
    // NorthEast: centro in (x + halfW/2, y + halfH/2)
    // SouthWest: centro in (x - halfW/2, y - halfH/2)
    // SouthEast: centro in (x + halfW/2, y - halfH/2)

    float quaterWidth = boundary.halfWidth / 2;
    float quaterHeight = boundary.halfHeight / 2;

    northWest = std::make_unique<QuadTree>(AABB(Vector2(boundary.center.x - quaterWidth, boundary.center.y + quaterHeight), quaterWidth, quaterHeight), capacity);
    northEast = std::make_unique<QuadTree>(AABB(Vector2(boundary.center.x + quaterWidth, boundary.center.y + quaterHeight), quaterWidth, quaterHeight), capacity);
    southWest = std::make_unique<QuadTree>(AABB(Vector2(boundary.center.x - quaterWidth, boundary.center.y - quaterHeight), quaterWidth, quaterHeight), capacity);
    southEast = std::make_unique<QuadTree>(AABB(Vector2(boundary.center.x + quaterWidth, boundary.center.y - quaterHeight), quaterWidth, quaterHeight), capacity);
    
    divided = true;
}