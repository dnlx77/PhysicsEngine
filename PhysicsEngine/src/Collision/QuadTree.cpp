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
            InsertIntoChildren(obj);
        }

        // Svuota questo nodo (ora gli oggetti sono nei figli)
        objects.clear();
    }

    return InsertIntoChildren(body);
}

void QuadTree::Query(const AABB &range, std::set<RigidBody *> &found)
{
    if (!boundary.Intersects(range))
        return;

    for (auto obj : objects) {
        if (range.Contains(obj->position))
            found.insert(obj);
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

bool QuadTree::InsertIntoChildren(RigidBody *body)
{
    // Calcola AABB del corpo
    float halfW = body->shapeType == ShapeType::CIRCLE ?
        body->radius : body->width / 2.0f;
    float halfH = body->shapeType == ShapeType::CIRCLE ?
        body->radius : body->height / 2.0f;

    AABB bodyBounds(body->position, halfW, halfH);

    // Inserisci in TUTTI i quadranti che interseca
    bool inserted = false;
    if (northWest->boundary.Intersects(bodyBounds))
        inserted |= northWest->Insert(body);
    if (northEast->boundary.Intersects(bodyBounds))
        inserted |= northEast->Insert(body);
    if (southWest->boundary.Intersects(bodyBounds))
        inserted |= southWest->Insert(body);
    if (southEast->boundary.Intersects(bodyBounds))
        inserted |= southEast->Insert(body);

    return inserted;
}