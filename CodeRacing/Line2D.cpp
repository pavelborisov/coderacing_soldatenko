#include "Line2D.h"

#include <cmath>
#include "math.h"
#include "assert.h"

static const double epsilon = 1e-7;

CLine2D::CLine2D(double A, double B, double C) : A(A), B(B), C(C)
{
	PseudoLength = hypot(A, B);
}

CLine2D CLine2D::FromPoints(const CVec2D& start, const CVec2D& end)
{
	return CLine2D(end.Y - start.Y, start.X - end.X, (start.Y - end.Y) * start.X + (end.X - start.X) * start.Y);
}

double CLine2D::GetSignedDistanceFrom(const CVec2D& point) const
{
	return (A * point.X + B * point.Y + C) / PseudoLength;
}

double CLine2D::GetDistanceFrom(const CVec2D& point) const
{
	return abs(GetSignedDistanceFrom(point));
}

double CLine2D::GetDistanceFrom(const CLine2D& line) const
{
	CVec2D intersection;
	if (GetIntersectionPoint(line, intersection)) {
		assert(false);
		return 0;
	}
	return abs(C - line.C) / PseudoLength;
}

bool CLine2D::GetIntersectionPoint(const CLine2D& line, CVec2D& result) const
{
	double d = A * line.B - line.A * B;
	if (abs(d) < abs(epsilon)) {
		return false;
	} else {
		result = CVec2D((B * line.C - line.B * C) / d, (line.A * C - A * line.C) / d);
		return true;
	}
}

CLine2D CLine2D::GetParallelLine(const CVec2D& point) const
{
	double shift = A * point.X + B * point.Y + C;
	return CLine2D(A, B, C - shift);
}

CVec2D CLine2D::GetUnitNormalFrom(const CVec2D& point) const
{
	double signedDistance = GetSignedDistanceFrom(point);

	if (signedDistance <= -epsilon) {
		return CVec2D(A / PseudoLength, B / PseudoLength);
	} else if (signedDistance >= epsilon) {
		return CVec2D(-A / PseudoLength, -B / PseudoLength);
	} else {
		assert(false);
		return CVec2D();
		//throw new IllegalArgumentException(String.format("Point {x=%s, y=%s} is on the %s.", x, y, this));
	}
}

CVec2D CLine2D::GetProjectionOf(const CVec2D& point) const
{
	double distance = abs(GetSignedDistanceFrom(point));
	if (distance < epsilon) {
		return point;
	}

	CVec2D unitNormal = GetUnitNormalFrom(point);
	return point + unitNormal * distance;
}

