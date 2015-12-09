#include "Vec3D.h"

#include <math.h>
#include <assert.h>
#include "Tools.h"

double CVec3D::Length() const
{
	return sqrt(LengthSquared());
}
