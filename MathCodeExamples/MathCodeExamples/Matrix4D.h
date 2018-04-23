#pragma once
#include "Vector4D.h"
#include "Vector3D.h"


struct Matrix4D
{
protected:
	float n[4][4];

public:

	Matrix4D() = default;

	Matrix4D(float n00, float n01, float n02, float n03,
		float n10, float n11, float n12, float n13,
		float n20, float n21, float n22, float n23,
		float n30, float n31, float n32, float n33)
	{
		n[0][0] = n00; n[0][1] = n10; n[0][2] = n20; n[0][3] = n30;
		n[1][0] = n01; n[1][1] = n11; n[1][2] = n21; n[1][3] = n31;
		n[2][0] = n02; n[2][1] = n12; n[2][2] = n22; n[2][3] = n32;
		n[3][0] = n03; n[3][1] = n13; n[3][2] = n23; n[3][3] = n33;
	}

	Matrix4D(const Vector4D& a, const Vector4D& b, const Vector4D& c, const Vector4D& d)
	{
		n[0][0] = a.x; n[0][1] = a.y; n[0][2] = a.z; n[0][3] = a.w;
		n[1][0] = b.x; n[1][1] = b.y; n[1][2] = b.z; n[1][3] = b.w;
		n[2][0] = c.x; n[2][1] = c.y; n[2][2] = c.z; n[2][3] = c.w;
		n[3][0] = d.x; n[3][1] = d.y; n[3][2] = d.z; n[3][3] = d.w;
	}

	float& operator ()(int i, int j)
	{
		return (n[i][j]);
	}

	const float& operator ()(int i, int j) const
	{
		return (n[i][j]);
	}

	Vector4D& operator [](int j) 
	{
		return (*reinterpret_cast<Vector4D *>(n[j]));
	}

	const Vector4D& operator [](int j)  const
	{
		return (*reinterpret_cast<const Vector4D *>(n[j]));
	}
};

//Inverse of a 4x4 matrix applying four column 3D vectors and one independent row perspective
Matrix4D Inverse(const Matrix4D& M)
{
	const Vector3D& a = reinterpret_cast<const Vector3D&>(M[0]);
	const Vector3D& b = reinterpret_cast<const Vector3D&>(M[1]);
	const Vector3D& c = reinterpret_cast<const Vector3D&>(M[2]);
	const Vector3D& d = reinterpret_cast<const Vector3D&>(M[3]);

	const float& x = M(3, 0);
	const float& y = M(3, 1);
	const float& z = M(3, 2);
	const float& w = M(3, 3);

	Vector3D s = Cross(a, b);
	Vector3D t = Cross(c, d);
	Vector3D u = a * y - b * x;
	Vector3D v = c * w - d * z;

	float invDet = 1.0F / (Dot(s, v) + Dot(t, u));
	s *= invDet;
	t *= invDet;
	u *= invDet;
	v *= invDet;

	Vector3D r0 = Cross(b, v) + t * y;
	Vector3D r1 = Cross(v, a) - t * x;
	Vector3D r2 = Cross(d, u) + s * w;
	Vector3D r3 = Cross(u, c) - s * z;

	return (Matrix4D(r0.x, r0.y, r0.z, -Dot(b, t),
		r1.x, r1.y, r1.z, Dot(a, t),
		r2.x, r2.y, r2.z, -Dot(d, s),
		r3.x, r3.y, r3.z, Dot(c, s)));
}