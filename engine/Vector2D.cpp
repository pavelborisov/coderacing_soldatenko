#include "Vector2D.h"

#include "Point2D.h"

namespace Engine {
	CVector2D::CVector2D(const CPoint2D& p1, const CPoint2D& p2)
	{
		SetX(p2.GetX() - p1.GetX());
		SetY(p2.GetY() - p1.GetY());
	}
} // namespace engine