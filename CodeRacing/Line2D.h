#pragma once

#include "Vec2D.h"

struct CLine2D {
	double A;
	double B;
	double C;
	double PseudoLength;

	CLine2D() : A(0), B(0), C(0), PseudoLength(0) {}
	CLine2D(double A, double B, double C);

	static CLine2D FromPoints(const CVec2D& start, const CVec2D& end);

	double GetSignedDistanceFrom(const CVec2D& point) const;
	double GetDistanceFrom(const CVec2D& point) const;
	double GetDistanceFrom(const CLine2D& line) const;
	bool GetIntersectionPoint(const CLine2D& line, CVec2D& result) const;
	CLine2D GetParallelLine(const CVec2D& point) const;
	CVec2D GetUnitNormal() const;
	CVec2D GetUnitNormalFrom(const CVec2D& point) const;
	CVec2D GetProjectionOf(const CVec2D& point) const;
};
