#include <iostream>
#include "Math/Vector2.h"
#include "Physics/RigidBody.h"
#include "Physics/PhysicsWorld.h"

void TestVector2()
{
    std::cout << "=== Test Vector2 ===" << std::endl;

    // Test costruttori
    Vector2 v1;                    // Default (0,0)
    Vector2 v2(3.0f, 4.0f);       // Parametrizzato
    Vector2 v3 = v2;              // Copy

    std::cout << "v1: (" << v1.x << ", " << v1.y << ")" << std::endl;
    std::cout << "v2: (" << v2.x << ", " << v2.y << ")" << std::endl;
    std::cout << "v3: (" << v3.x << ", " << v3.y << ")" << std::endl;

    // Test operatori
    Vector2 v4 = v2 + v3;
    std::cout << "v2 + v3: (" << v4.x << ", " << v4.y << ")" << std::endl;

    // Test lunghezza (dovrebbe essere 5 per (3,4))
    std::cout << "Length v2: " << v2.Length() << std::endl;

    // Test normalizzazione
    Vector2 v5 = v2.Normalized();
    std::cout << "v2 normalized: (" << v5.x << ", " << v5.y << ")" << std::endl;
    std::cout << "Length normalized: " << v5.Length() << std::endl;

    // Test rotazione (90 gradi = π/2 radianti)
    Vector2 v6 = Vector2::RIGHT.Rotated(3.14159f / 2.0f);
    std::cout << "RIGHT rotated 90°: (" << v6.x << ", " << v6.y << ")" << std::endl;

    // Test costanti
    std::cout << "ZERO: (" << Vector2::ZERO.x << ", " << Vector2::ZERO.y << ")" << std::endl;
    std::cout << "UP: (" << Vector2::UP.x << ", " << Vector2::UP.y << ")" << std::endl;

    std::cout << "=== Test completati ===" << std::endl;
}

void TestRigidBody()
{
    std::cout << "\n=== Test RigidBody ===" << std::endl;

    // Crea un corpo rigido
    RigidBody body(Vector2(0, 10), 2.0f);  // Massa 2kg, posizione (0,10)

    std::cout << "Posizione iniziale: (" << body.position.x << ", " << body.position.y << ")" << std::endl;
    std::cout << "Massa: " << body.mass << ", Massa inversa: " << body.inverseMass << std::endl;

    // Simula gravità per 1 secondo
    Vector2 gravity(0, -9.8f);  // 9.8 m/s² verso il basso
    float deltaTime = 0.016f;   // 60 FPS

    for (int i = 0; i < 60; i++) {  // 1 secondo di simulazione
        body.ApplyForce(gravity * body.mass);  // F = mg
        body.Integrate(deltaTime);
        body.ClearForces();
    }

    std::cout << "Dopo 1s di caduta:" << std::endl;
    std::cout << "Posizione: (" << body.position.x << ", " << body.position.y << ")" << std::endl;
    std::cout << "Velocità: (" << body.velocity.x << ", " << body.velocity.y << ")" << std::endl;

    // Test rotazione
    RigidBody rotatingBody(Vector2::ZERO, 1.0f);
    rotatingBody.ApplyTorque(5.0f);  // Momento torcente
    rotatingBody.Integrate(0.1f);
    rotatingBody.ClearForces();

    std::cout << "Test rotazione - Angolo: " << rotatingBody.angle << " rad" << std::endl;
}

void TestRigidBodyAdvanced()
{
    std::cout << "\n=== Test RigidBody Avanzati ===" << std::endl;

    // Test 1: Oggetto statico
    std::cout << "\n--- Test 1: Oggetto Statico ---" << std::endl;
    RigidBody staticBody(Vector2(0, 0), 0.0f);  // Massa 0 = statico
    std::cout << "Oggetto statico - IsStatic: " << (staticBody.IsStatic() ? "true" : "false") << std::endl;

    staticBody.ApplyForce(Vector2(100, 100));  // Forza enorme
    staticBody.Integrate(1.0f);  // 1 secondo
    staticBody.ClearForces();

    std::cout << "Dopo forza enorme - Posizione: (" << staticBody.position.x << ", " << staticBody.position.y << ")" << std::endl;
    std::cout << "Velocità: (" << staticBody.velocity.x << ", " << staticBody.velocity.y << ")" << std::endl;

    // Test 2: Lancio parabolico
    std::cout << "\n--- Test 2: Lancio Parabolico ---" << std::endl;
    RigidBody projectile(Vector2(0, 0), 1.0f);
    projectile.SetVelocity(Vector2(10, 15));  // Velocità iniziale diagonale

    std::cout << "Velocità iniziale: (10, 15)" << std::endl;

    Vector2 gravity(0, -9.8f);
    float deltaTime = 0.016f;

    // Simula fino a quando non tocca il suolo (y <= 0)
    int steps = 0;
    while (projectile.position.y >= 0 && steps < 300) {  // Max 5 secondi
        projectile.ApplyForce(gravity * projectile.mass);
        projectile.Integrate(deltaTime);
        projectile.ClearForces();
        steps++;
    }

    float timeOfFlight = steps * deltaTime;
    std::cout << "Tempo di volo: " << timeOfFlight << " secondi" << std::endl;
    std::cout << "Posizione finale: (" << projectile.position.x << ", " << projectile.position.y << ")" << std::endl;
    std::cout << "Velocità finale: (" << projectile.velocity.x << ", " << projectile.velocity.y << ")" << std::endl;

    // Test 3: Rotazione con momento variabile
    std::cout << "\n--- Test 3: Rotazione ---" << std::endl;
    RigidBody spinner(Vector2::ZERO, 1.0f);
    spinner.SetInertia(2.0f);  // Inerzia diversa dalla massa

    std::cout << "Inerzia: " << spinner.inertia << ", Inerzia inversa: " << spinner.inverseInertia << std::endl;

    // Applica momento per mezzo secondo
    for (int i = 0; i < 30; i++) {  // 0.5 secondi
        spinner.ApplyTorque(10.0f);
        spinner.Integrate(0.016f);
        spinner.ClearForces();
    }

    std::cout << "Dopo 0.5s di momento costante:" << std::endl;
    std::cout << "Velocità angolare: " << spinner.angularVelocity << " rad/s" << std::endl;
    std::cout << "Angolo totale: " << spinner.angle << " rad (" << (spinner.angle * 180.0f / 3.14159f) << " gradi)" << std::endl;

    // Test 4: Massa molto piccola vs molto grande
    std::cout << "\n--- Test 4: Masse Estreme ---" << std::endl;
    RigidBody lightBody(Vector2(0, 10), 0.01f);   // 10 grammi
    RigidBody heavyBody(Vector2(0, 10), 1000.0f); // 1 tonnellata

    Vector2 sameForce(0, -10.0f);  // Stessa forza su entrambi

    for (int i = 0; i < 60; i++) {  // 1 secondo
        lightBody.ApplyForce(sameForce);
        heavyBody.ApplyForce(sameForce);

        lightBody.Integrate(0.016f);
        heavyBody.Integrate(0.016f);

        lightBody.ClearForces();
        heavyBody.ClearForces();
    }

    std::cout << "Corpo leggero (0.01kg) - Velocità finale: " << lightBody.velocity.y << std::endl;
    std::cout << "Corpo pesante (1000kg) - Velocità finale: " << heavyBody.velocity.y << std::endl;
    std::cout << "Rapporto velocità (dovrebbe essere ~100000): " << (lightBody.velocity.y / heavyBody.velocity.y) << std::endl;
}

void TestPhysicsWorld()
{
    PhysicsWorld world;

    RigidBody *ball = world.CreateRigidBody(Vector2(0, 10), 2.0f);

    // Simula 1 secondo
    for (int i = 0; i < 60; i++) {
        world.Update(1.0f / 60.0f);
    }

    std::cout << "Ball position: (" << ball->position.x << ", " << ball->position.y << ")" << std::endl;
}

int main()
{
    TestVector2();
    TestRigidBody();
    TestRigidBodyAdvanced();
    TestPhysicsWorld();
    return 0;
}