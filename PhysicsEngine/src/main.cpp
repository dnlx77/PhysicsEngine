#include <iostream>
#include "Math/Vector2.h"

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

int main()
{
    TestVector2();
    return 0;
}