#include "Math/Vector2.h"
#include <cmath>
#include <stdexcept>

// Definizione delle costanti statiche
const Vector2 Vector2::ZERO(0.0f, 0.0f);
const Vector2 Vector2::ONE(1.0f, 1.0f);
const Vector2 Vector2::UP(0.0f, 1.0f);
const Vector2 Vector2::DOWN(0.0f, -1.0f);
const Vector2 Vector2::RIGHT(1.0f, 0.0f);
const Vector2 Vector2::LEFT(-1.0f, 0.0f);

// Costruttori
Vector2::Vector2() : x(0.0f), y(0.0f)
{
}

Vector2::Vector2(float x, float y) : x(x), y(y)
{
}

Vector2::Vector2(const Vector2 &other) : x(other.x), y(other.y)
{
}

// Operatori matematici
Vector2 Vector2::operator+(const Vector2 &other) const
{
	return Vector2(x + other.x, y + other.y);
}

Vector2 Vector2::operator-(const Vector2 &other) const
{
	return Vector2(x - other.x, y - other.y);
}

Vector2 Vector2::operator*(const Vector2 &other) const
{
	return Vector2(x * other.x, y * other.y);
}

Vector2 Vector2::operator/(const Vector2 &other) const
{
	if (std::abs(other.x) < 1e-6f || std::abs(other.y) < 1e-6f)
		throw std::invalid_argument("Division by zero in Vector2");

	return Vector2(x / other.x, y / other.y);
}

Vector2 Vector2::operator*(float scalar) const
{
	return Vector2(x * scalar, y * scalar);
}

Vector2 Vector2::operator/(float scalar) const
{
	if (std::abs(scalar) < 1e-6f)
		throw std::invalid_argument("Division by zero in Vector2");
	return Vector2(x / scalar, y / scalar);
}

bool Vector2::operator==(const Vector2 &other) const
{
	return (std::abs(x - other.x) < 1e-6f && std::abs(y - other.y) < 1e-6f);
}

bool Vector2::operator!=(const Vector2 &other) const
{
	return !(*this == other);
}

Vector2 &Vector2::operator+=(const Vector2 &other)
{
	x += other.x;
	y += other.y;
	return *this;  // Ritorna riferimento a se stesso
}

Vector2 &Vector2::operator-=(const Vector2 &other)
{
	x -= other.x;
	y -= other.y;
	return *this;  // Ritorna riferimento a se stesso
}

Vector2 &Vector2::operator*=(const Vector2 &other)
{
	x *= other.x;
	y *= other.y;
	return *this;  // Ritorna riferimento a se stesso
}

Vector2 &Vector2::operator/=(const Vector2 &other)
{
	if (std::abs(other.x) < 1e-6f || std::abs(other.y) < 1e-6f)
		throw std::invalid_argument("Division by zero in Vector2");

	x /= other.x;
	y /= other.y;
	return *this;  // Ritorna riferimento a se stesso
}

Vector2 &Vector2::operator*=(float scalar)
{
	x *= scalar;
	y *= scalar;
	return *this;
}

Vector2 &Vector2::operator/=(float scalar)
{
	if (std::abs(scalar) < 1e-6f)
		throw std::invalid_argument("Division by zero in Vector2");

	x /= scalar;
	y /= scalar;
	return *this;
}

float &Vector2::operator[](int index)
{
	if (index == 0) return x;
	if (index == 1) return y;
	throw std::out_of_range("Vector2 index is out of range");
}

const float &Vector2::operator[](int index) const
{
	if (index == 0) return x;
	if (index == 1) return y;
	throw std::out_of_range("Vector2 index is out of range");
}

Vector2 &Vector2::Normalize()
{
	float len = Length();
	if (len < 1e-6f) {
		x = 0.0f;
		y = 0.0f;
	}
	else
	{
		x /= len;
		y /= len;
	}

	return *this;
}

Vector2 &Vector2::Rotate(float angleRadians)
{
	float cos_a = std::cos(angleRadians);
	float sin_a = std::sin(angleRadians);
	float new_x = x * cos_a - y * sin_a;
	float new_y = x * sin_a + y * cos_a;
	x = new_x;
	y = new_y;
	return *this;
}

Vector2 Vector2::Normalized() const
{
	float len = Length();
	if (len < 1e-6f)
		return Vector2::ZERO;

	return Vector2(x / len, y / len);
}

Vector2 Vector2::Rotated(float angleRadians) const
{
	float cos_a = std::cos(angleRadians);
	float sin_a = std::sin(angleRadians);
	return Vector2(
		x * cos_a - y * sin_a,
		x * sin_a + y * cos_a
	);
}

// Metodi di calcolo
float Vector2::LengthSquared() const
{
	return x * x + y * y;
}

float Vector2::Length() const
{
	return std::sqrt(LengthSquared());
}

float Vector2::Dot(const Vector2 &other) const
{
	return x * other.x + y * other.y;
}

// Metodi statici
float Vector2::Distance(const Vector2 &a, const Vector2 &b)
{
	return (b - a).Length();
}

float Vector2::DistanceSquared(const Vector2 &a, const Vector2 &b)
{
	return (b - a).LengthSquared();
}

Vector2 Vector2::Lerp(const Vector2 &a, const Vector2 &b, float t)
{
	return a + t * (b - a);
}

// Operatore globale

Vector2 operator*(float scalar, const Vector2 &vector)
{
	return vector * scalar;}
