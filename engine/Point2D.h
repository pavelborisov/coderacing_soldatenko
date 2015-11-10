#pragma once

#include <math.h>
#include <string>

// TODO: Возможна циклическая зависимость, сделать forward declaration и спрятать методы с CVector2D в .cpp-шник
#include "Vector2D.h"

namespace Engine {

	class CPoint2D {
	public:
		CPoint2D() : x(0), y(0) {}
		CPoint2D(double _x, double _y) : x(_x), y(_y) {}
		CPoint2D(const CPoint2D& point) : x(point.x), y(point.y) {}

		double GetX() const { return x; }
		void SetX(double _x) { x = _x; }
		double GetY() const { return y; };
		void SetY(double _y) { y = _y; }

		CPoint2D& Add(const CVector2D& vector)
		{
			SetX(GetX() + vector.GetX());
			SetY(GetY() + vector.GetY());
			return *this;
		}

		CPoint2D& Add(double x, double y)
		{
			SetX(GetX() + x);
			SetY(GetY() + y);
			return *this;
		}

		CPoint2D& Subtract(const CVector2D& vector)
		{
			SetX(GetX() - vector.GetX());
			SetY(GetY() - vector.GetY());
			return *this;
		}

		CPoint2D& Subtract(double x, double y)
		{
			SetX(GetX() - x);
			SetY(GetY() - y);
			return *this;
		}

		double GetDistanceTo(const CPoint2D& point) const
		{
			return hypot(GetX() - point.GetX(), GetY() - point.GetY());
		}

		double GetDistanceTo(double x, double y) const
		{
			return hypot(GetX() - x, GetY() - y);
		}

		double GetSquaredDistanceTo(const CPoint2D& point) const
		{
			return pow(GetX() - point.GetX(), 2) + pow(GetY() - point.GetY(), 2);
		}

		double GetSquaredDistanceTo(double x, double y) const
		{
			return pow(GetX() - x, 2) + pow(GetY() - y, 2);;
		}

		//@Contract(value = "-> !null", pure = true)
		//	public Point2D copy() {
		//	return new Point2D(this);
		//}

		bool NearlyEquals(const CPoint2D& point, double epsilon) const
		{
			return abs(GetX() - point.GetX() < epsilon)
				&& abs(GetY() - point.GetY() < epsilon);
		}

		bool NearlyEquals(const CPoint2D& point) const
		{
			// TODO: DEFAULT_EPSILON
			assert(false);
			double DEFAULT_EPSILON = 1e-10;
			return NearlyEquals(point, DEFAULT_EPSILON);
		}

		bool NearlyEquals(double x, double y, double epsilon) const
		{
			return abs(GetX() - x < epsilon)
				&& abs(GetY() - y < epsilon);
		}

		bool NearlyEquals(double x, double y) const
		{
			// TODO: DEFAULT_EPSILON
			assert(false);
			double DEFAULT_EPSILON = 1e-10;
			return NearlyEquals(x, y, DEFAULT_EPSILON);
		}

		std::string ToString() {
			return std::to_string(x) + "," + std::to_string(y);
		}
	private:
		double x;
		double y;

	};

} // namespace Engine