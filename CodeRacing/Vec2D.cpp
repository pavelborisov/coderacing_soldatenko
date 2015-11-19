#include "Vec2D.h"

#include <math.h>
#include <assert.h>
#include "Tools.h"

double CVec2D::Length() const
{
	return hypot(X, Y);
}

double CVec2D::GetAngle() const
{
	assert(LengthSquared() > 0);
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