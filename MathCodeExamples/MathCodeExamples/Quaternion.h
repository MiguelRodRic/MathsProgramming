#pragma once
#include "Vector3D.h"
#include "Matrix3D.h"

struct Quaternion 
{
	float x, y, z, w;

	Quaternion() = default;

	Quaternion(float a, float b, float c, float s)
	{
		//Vector part
		x = a; 
		y = b; 
		z = c;

		//Scalar part
		w = s;
	}

	Quaternion(const Vector3D& v, float s) 
	{
		//Vector part
		x = v.x;
		y = v.y;
		z = v.z;

		//Scalar part
		w = s;
	}

	const Vector3D& GetVectorPart(void) const
	{
		return (reinterpret_cast<const Vector3D&>(x));
	}

	Matrix3D GetRotationMatrix(void);

	void SetRotationMatrix(const Matrix3D& m);
};

Quaternion operator *(const Quaternion& q1, const Quaternion& q2)
{
	return (Quaternion(
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
		q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.y
	));
}