#include "Point2D.h"

#include "Vector2D.h"

namespace Engine {
	CPoint2D& CPoint2D::Add(const CVector2D& vector)
	{
		SetX(GetX() + vector.GetX());
		SetY(GetY() + vector.GetY());
		return *this;
	}

	CPoint2D& CPoint2D::Subtract(const CVector2D& vector)
	{
		SetX(GetX() - vector.GetX());
		SetY(GetY() - vector.GetY());
		return *this;
	}
} // namespace Engine