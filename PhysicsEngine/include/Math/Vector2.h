#pragma once
#include <cmath>

class Vector2 {
public:
	// Membri pubblici
	float x, y;

	// Costruttori
	Vector2();
	Vector2(float x, float y);
	Vector2(const Vector2 &other);

	// Operatori matematici con altri Vector2
	Vector2 operator+(const Vector2 &other) const;
	Vector2 operator-(const Vector2 &other) const;
	Vector2 operator*(const Vector2 &other) const;
	Vector2 operator/(const Vector2 &other) const;

	// Operatori matematici con scalari
	Vector2 operator*(float scalar) const;
	Vector2 operator/(float scalar) const;

	// Operatori di confronto
	bool operator==(const Vector2 &other) const;
	bool operator!=(const Vector2 &other) const;

	// Operatori compound (modificano l'oggetto)
	Vector2 &operator+=(const Vector2 &other);
	Vector2 &operator-=(const Vector2 &other);
	Vector2 &operator*=(const Vector2 &other);
	Vector2 &operator/=(const Vector2 &other);
	Vector2 &operator*=(float scalar);
	Vector2 &operator/=(float scalar);

	// Operatori di accesso per indice
	float &operator[](int index);
	const float &operator[](int index) const;

	// Metodi che modificano l'oggetto corrente
	Vector2 &Normalize();
	Vector2 &Rotate(float angleRadians);

	// Metodi che ritornano nuovo oggetto
	Vector2 Normalized() const;
	Vector2 Rotated(float angleRadians) const;

	// Metodi di calcolo
	float Length() const;
	float LengthSquared() const;
	float Dot(const Vector2 &other) const;

	// Metodi statici
	static float Distance(const Vector2 &a, const Vector2 &b);
	static float DistanceSquared(const Vector2 &a, const Vector2 &b);
	static Vector2 Lerp(const Vector2 &a, const Vector2 &b, float t);

	// Costanti statiche
	static const Vector2 ZERO;
	static const Vector2 ONE;
	static const Vector2 UP;
	static const Vector2 RIGHT;

};

// Operatore globale per moltiplicazione scalare a sinistra
Vector2 operator*(float scalar, const Vector2 &vector);
