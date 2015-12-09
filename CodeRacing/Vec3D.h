#pragma once

#include "Vec2D.h"

struct CVec3D {
	double X;
	double Y;
	double Z;

	CVec3D() : X(0), Y(0), Z(0) {}
	CVec3D(double _X, double _Y, double _Z) : X(_X), Y(_Y), Z(_Z) {}
	CVec3D(const CVec3D& v) : X(v.X), Y(v.Y), Z(v.Z) {}
	explicit CVec3D(const CVec2D& v) : X(v.X), Y(v.Y), Z(0) {}
	explicit CVec3D(double _Z) : X(0), Y(0), Z(_Z) {}

	CVec3D& operator = (const CVec3D& v);
	bool operator == (const CVec3D& v) const;
	bool operator != (const CVec3D& v) const;
	CVec3D operator - () const;
	CVec3D operator + (const CVec3D& v) const;
	CVec3D operator - (const CVec3D& v) const;
	CVec3D operator * (double scalar) const;
	CVec3D& operator += (const CVec3D& v);
	CVec3D& operator -= (const CVec3D& v);
	CVec3D& operator *= (double scalar);

	double DotProduct(const CVec3D& v) const;
	double Length() const;
	double LengthSquared() const;

	CVec3D Cross(const CVec3D& v) const;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline CVec3D& CVec3D::operator = (const CVec3D& v)
{
	X = v.X;
	Y = v.Y;
	Z = v.Z;
	return *this;
}

inline bool CVec3D::operator == (const CVec3D& v) const
{
	return X == v.X && Y == v.Y && Z == v.Z;
}

inline bool CVec3D::operator != (const CVec3D& v) const
{
	return X != v.X || Y != v.Y || Z != v.Z;
}

inline CVec3D CVec3D::operator - () const
{
	return CVec3D(-X, -Y, -Z);
}

inline CVec3D CVec3D::operator + (const CVec3D& v) const
{
	return CVec3D(X + v.X, Y + v.Y, Z + v.Z);
}

inline CVec3D CVec3D::operator - (const CVec3D& v) const
{
	return CVec3D(X - v.X, Y - v.Y, Z - v.Z);
}

inline CVec3D CVec3D::operator * (double scalar) const
{
	return CVec3D(X * scalar, Y * scalar, Z * scalar);
}

inline CVec3D& CVec3D::operator += (const CVec3D& v)
{
	X += v.X;
	Y += v.Y;
	Z += v.Z;
	return *this;
}

inline CVec3D& CVec3D::operator -= (const CVec3D& v)
{
	X -= v.X;
	Y -= v.Y;
	Z -= v.Z;
	return *this;
}

inline CVec3D& CVec3D::operator *= (double scalar)
{
	X *= scalar;
	Y *= scalar;
	Z *= scalar;
	return *this;
}

inline double CVec3D::DotProduct(const CVec3D& v) const
{
	return X * v.X + Y * v.Y + Z * v.Z;
}

inline double CVec3D::LengthSquared() const
{
	return X * X + Y * Y + Z * Z;
}

inline CVec3D CVec3D::Cross(const CVec3D& v) const
{
	CVec3D result;
	result.X = Y * v.Z - Z * v.Y;
	result.Y = Z * v.X - X * v.Z;
	result.Z = X * v.Y - Y * v.X;
	return result;
}
