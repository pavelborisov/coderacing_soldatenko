#pragma once

#include "Vec2D.h"

struct CArc2D {
	CVec2D Center;
	double Radius;
	double StartAngle;
	double FinishAngle;
	CVec2D Point1;
	CVec2D Point2;

	CArc2D() : Radius(0), StartAngle(0), FinishAngle(0) {}
	CArc2D(const CVec2D& Center, double Radius, double StartAngle, double SectorAngle);
};
