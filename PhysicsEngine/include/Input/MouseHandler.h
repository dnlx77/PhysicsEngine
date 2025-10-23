#pragma once
#include "Math/Vector2.h"
#include "Physics/RigidBody.h"
#include "Physics/PhysicsWorld.h"

class MouseHandler {
private:
    Vector2 mouseWorldPosition;      // Posizione corrente del mouse nel mondo
    RigidBody *selectedBody;         // Corpo attualmente selezionato
    Vector2 clickOffsetLocal;        // Offset dal centro del corpo al punto di click

    float dragStiffness;             // Costante elastica della "molla" del drag
    float dragDamping;

    // Helper privato: trova quale corpo è sotto il cursore
    RigidBody *FindBodyAtPosition(const Vector2 &worldPos, PhysicsWorld &world);

public:
    MouseHandler(float stiffness = 500.0f, float damping = 10.0f);  // Costruttore con stiffness regolabile

    // Gestione eventi
    void HandleMousePress(const Vector2 &worldPos, PhysicsWorld &world);
    void HandleMouseMove(const Vector2 &worldPos);
    void HandleMouseRelease();

    // Applica forze al corpo selezionato (chiamato ogni frame)
    void Update(PhysicsWorld &);

    // Query
    bool IsDragging() const { return selectedBody != nullptr; }
    RigidBody *GetSelectedBody() const { return selectedBody; }

    // Getter
    Vector2 GetMouseWorldPosition() const { return mouseWorldPosition; }
    Vector2 GetAttachPoint() const;
};