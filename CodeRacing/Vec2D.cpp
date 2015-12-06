#include "Vec2D.h"

#include <cmath>
#include <math.h>
#include <assert.h>
#include "Tools.h"

static const double epsilon = 1e-7;

double CVec2D::Length() const
{
	return hypot(X, Y);
}

double CVec2D::GetAngle() const
{
	//assert(LengthSquared() > 0);
	if (X == 0 && Y == 0) {
		return 0;
	}
	return atan2(Y, X);
}

void CVec2D::Rotate(double angle)
{
	const double sinValue = sin(angle);
	const double cosValue = cos(angle);
	const double xNew = X * cosValue - Y * sinValue;
	const double yNew = X * sinValue + Y * cosValue;
	X = xNew;
	Y = yNew;
}

void CVec2D::Normalize()
{
	*this *= 1.0 / Length();
}

bool CVec2D::NearlyEquals(const CVec2D& v) const
{
	return abs(X - v.X) < epsilon && abs(Y - v.Y) < epsilon;
}
