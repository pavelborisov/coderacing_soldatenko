#pragma once

struct CVec2D {
	double X;
	double Y;

	CVec2D() : X(0), Y(0) {}
	CVec2D(double _X, double _Y) : X(_X), Y(_Y) {}
	CVec2D(const CVec2D& v) : X(v.X), Y(v.Y) {}

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
	double GetAngleTo(const CVec2D& v) const;
	void Rotate(double angle);
	
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
