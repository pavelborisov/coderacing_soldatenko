#pragma once

struct CVec2D {
	double X;
	double Y;

	CVec2D() : X(0), Y(0) {}
	CVec2D(double _X, double _Y) : X(_X), Y(_Y) {}
	CVec2D(const CVec2D& v) : X(v.X), Y(v.Y) {}
	CVec2D(const CVec2D& v1, const CVec2D& v2) : X(v2.X - v1.X), Y(v2.Y - v1.Y) {}

	CVec2D& operator = (const CVec2D& v);
	bool operator == (const CVec2D& v) const;
	bool operator != (const CVec2D& v) const;
	CVec2D operator - () const;
	CVec2D operator + (const CVec2D& v) const;
	CVec2D operator - (const CVec2D& v) const;
	CVec2D operator * (double scalar) const;
	CVec2D& operator += (const CVec2D& v);
	CVec2D& operator -= (const CVec2D& v);
	CVec2D& operator *= (double scalar);

	double DotProduct(const CVec2D& v) const;
	double Length() const;
	double LengthSquared() const;

	// [-pi, pi]
	double GetAngle() const;
	void Rotate(double angle);

	bool NearlyEquals(const CVec2D& v) const;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline CVec2D& CVec2D::operator = (const CVec2D& v)
{
	X = v.X;
	Y = v.Y;
	return *this;
}

inline bool CVec2D::operator == (const CVec2D& v) const
{
	return X == v.X && Y == v.Y;
}

inline bool CVec2D::operator != (const CVec2D& v) const
{
	return X != v.X || Y != v.Y;
}

inline CVec2D CVec2D::operator - () const
{
	return CVec2D(-X, -Y);
}

inline CVec2D CVec2D::operator + (const CVec2D& v) const
{
	return CVec2D(X + v.X, Y + v.Y);
}

inline CVec2D CVec2D::operator - (const CVec2D& v) const
{
	return CVec2D(X - v.X, Y - v.Y);
}

inline CVec2D CVec2D::operator * (double scalar) const
{
	return CVec2D(X * scalar, Y * scalar);
}

inline CVec2D& CVec2D::operator += (const CVec2D& v)
{
	X += v.X;
	Y += v.Y;
	return *this;
}

inline CVec2D& CVec2D::operator -= (const CVec2D& v)
{
	X -= v.X;
	Y -= v.Y;
	return *this;
}

inline CVec2D& CVec2D::operator *= (double scalar)
{
	X *= scalar;
	Y *= scalar;
	return *this;
}

inline double CVec2D::DotProduct(const CVec2D& v) const
{
	return X * v.X + Y * v.Y;
}

inline double CVec2D::LengthSquared() const
{
	return X * X + Y * Y;
}
