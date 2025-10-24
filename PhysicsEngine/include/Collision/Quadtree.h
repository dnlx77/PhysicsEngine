#pragma once
#include "Collision/AABB.h"
#include "Physics/RigidBody.h"
#include <vector>
#include <set>
#include <memory>

class QuadTree {
private:
    AABB boundary;
    int capacity;
    std::vector<RigidBody *> objects;

    bool divided;
    std::unique_ptr<QuadTree> northWest;
    std::unique_ptr<QuadTree> northEast;
    std::unique_ptr<QuadTree> southWest;
    std::unique_ptr<QuadTree> southEast;

    void Subdivide();
    bool InsertIntoChildren(RigidBody *body);

public:
    QuadTree(const AABB &boundary, int capacity);

    bool Insert(RigidBody *body);
    void Query(const AABB &range, std::set<RigidBody *> &found);
    void Clear();
};
