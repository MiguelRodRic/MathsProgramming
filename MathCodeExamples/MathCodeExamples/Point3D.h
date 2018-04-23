#pragma once
#include "Vector3D.h"

struct Point3D : Vector3D
{
	Point3D() = default;

	Point3D(float a, float b, float c) : Vector3D(a, b, c) {}
};

inline Point3D operator +(const Point3D& a, const Vector3D& b) {
	return (Point3D(a.x + b.x, a.y + b.y, a.z + b.z));
}

inline Vector3D operator -(const Point3D& a, const Point3D& b)
{
	return (Vector3D(a.x - b.x, a.y - b.y, a.z - b.z));
}