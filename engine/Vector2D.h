#pragma once

#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <assert.h>

namespace Engine {

	class CPoint2D;

	class CVector2D {
	public:
		CVector2D() : x(0), y(0) {}
		CVector2D(double _x, double _y) : x(_x), y(_y) {}
		CVector2D(double x1, double y1, double x2, double y2) : x(x2 - x1), y(y2 - y1) {}
		CVector2D(const CVector2D& v) : x(v.x), y(v.y) {}
		CVector2D(const CPoint2D& p1, const CPoint2D& p2);

		double GetX() const { return x; }
		double GetY() const { return y; }
		void SetX(double _x) { x = _x; }
		void SetY(double _y) { y = _y; }

		CVector2D& Add(const CVector2D& v)
		{
			SetX(GetX() + v.GetX());
			SetY(GetY() + v.GetY());
			return *this;
		}

		CVector2D& Add(double x, double y)
		{
			SetX(GetX() + x);
			SetY(GetY() + y);
			return *this;
		}

		CVector2D& Subtract(const CVector2D& v)
		{
			SetX(GetX() - v.GetX());
			SetY(GetY() - v.GetY());
			return *this;
		}

		CVector2D& Subtract(double x, double y)
		{
			SetX(GetX() - x);
			SetY(GetY() - y);
			return *this;
		}

		CVector2D& Multiply(double factor) {
			SetX(factor * GetX());
			SetY(factor * GetY());
			return *this;
		}

		CVector2D& Rotate(double angle)
		{
			double c = cos(angle);
			double s = sin(angle);

			double x = GetX();
			double y = GetY();

			SetX(x * c - y * s);
			SetY(x * s + y * c);

			return *this;
		}

		double DotProduct(const CVector2D& v) const
		{
			return GetX() * v.GetX() + GetY() * v.GetY();
		}

		CVector2D& Negate()
		{
			SetX(-GetX());
			SetY(-GetY());
			return *this;
		}

		CVector2D& Normalize()
		{
			double length = GetLength();
			if (length == 0.0) {
				//throw new IllegalStateException("Can't set angle of zero-width vector.");
				assert(false);
			}
			SetX(GetX() / length);
			SetY(GetY() / length);
			return *this;
		}

		double GetAngle() const
		{
			return atan2(GetY(), GetX());
		}

		CVector2D& SetAngle(double angle)
		{
			double length = GetLength();
			if (length == 0.0) {
				//throw new IllegalStateException("Can't set angle of zero-width vector.");
				assert(false);
			}
			SetX(cos(angle) * length);
			SetY(sin(angle) * length);
			return *this;
		}

		double GetAngle(const CVector2D& vector) const
		{
			//return org.apache.commons.math3.geometry.euclidean.twod.Vector2D.angle(
			//	new org.apache.commons.math3.geometry.euclidean.twod.Vector2D(getX(), getY()),
			//	new org.apache.commons.math3.geometry.euclidean.twod.Vector2D(vector.getX(), vector.getY())
			//	);
			double dot = GetX() * vector.GetX() + GetY() * vector.GetY();
			double det = GetX() * vector.GetY() - GetY() * vector.GetX();
			double angle = atan2(det, dot);
			if (angle > M_PI) {
				angle -= 2 * M_PI;
			}
			if (angle < -M_PI) {
				angle += 2 * M_PI;
			}
			return angle;
		}

		double GetLength() const
		{
			return hypot(GetX(), GetY());
		}

		CVector2D& SetLength(double length)
		{
			double currentLength = GetLength();
			if (currentLength == 0.0) {
				//throw new IllegalStateException("Can't resize zero-width vector.");
				assert(false);
			}
			return Multiply(length / currentLength);
		}

		double GetSquaredLength() const
		{
			return GetX() * GetX() + GetY() * GetY();
		}

		CVector2D& SetSquaredLength(double squaredLength)
		{
			double currentSquaredLength = GetSquaredLength();
			if (currentSquaredLength == 0.0) {
				//throw new IllegalStateException("Can't resize zero-width vector.");
				assert(false);
			}
			return Multiply(sqrt(squaredLength / currentSquaredLength));
		}

		// TODO: copy
		//@Contract(value = "-> !null", pure = true)
		//	public Vector2D copy() {
		//	return new Vector2D(this);
		//}

		//@Contract(value = "-> !null", pure = true)
		//	public Vector2D copyNegate() {
		//	return new Vector2D(-getX(), -getY());
		//}

		bool NearlyEquals(const CVector2D& vector, double epsilon) const
		{
			return abs(GetX() - vector.GetX() < epsilon)
				&& abs(GetY() - vector.GetY() < epsilon);
		}

		bool NearlyEquals(const CVector2D& vector) const
		{
			// TODO: DEFAULT_EPSILON
			assert(false);
			double DEFAULT_EPSILON = 1e-10;
			return NearlyEquals(vector, DEFAULT_EPSILON);
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

		std::string ToString() const
		{
			return std::to_string(x) + "," + std::to_string(y);
		}

	private:
		double x;
		double y;

	};

} // namespace Engine