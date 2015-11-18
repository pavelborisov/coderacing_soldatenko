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

double CVec2D::GetAngleTo(const CVec2D& v) const
{
	const double thisAngle = GetAngle();
	const double otherAngle = v.GetAngle();
	double difAngle = otherAngle - thisAngle;
	if (difAngle > PI_2) {
		return difAngle - PI;
	} else if (difAngle < -PI_2) {
		return difAngle + PI;
	}
	return difAngle;
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