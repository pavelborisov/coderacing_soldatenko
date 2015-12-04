#include "Arc2D.h"

#include "assert.h"
#include "math.h"
#include "Tools.h"

CArc2D::CArc2D(const CVec2D& Center, double Radius, double StartAngle, double FinishAngle) :
	Center(Center), Radius(Radius), StartAngle(StartAngle), FinishAngle(FinishAngle)
{
	assert(Radius > 0);
	assert(-PI <= StartAngle && StartAngle <= PI);
	assert(-PI <= FinishAngle && FinishAngle <= PI);
	assert(StartAngle < FinishAngle);
	Point1 = Center + CVec2D(cos(StartAngle), sin(StartAngle)) * Radius;
	Point2 = Center + CVec2D(cos(FinishAngle), sin(FinishAngle)) * Radius;
}
