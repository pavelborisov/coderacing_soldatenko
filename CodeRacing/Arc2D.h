#pragma once

#include "Vec2D.h"

struct CArc2D {
	CVec2D Center;
	double Radius;
	double StartAngle;
	double SectorAngle;

	CArc2D() : Radius(0), StartAngle(0), SectorAngle(0) {}
	CArc2D(const CVec2D& Center, double Radius, double StartAngle, double SectorAngle) :
		Center(Center), Radius(Radius), StartAngle(StartAngle), SectorAngle(SectorAngle) {}
};
