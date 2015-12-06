#pragma once

#include "model/car.h"
#include "Vec2D.h"

const double UndefinedMedianAngularSpeed = -123456;

struct CRotatedRect {
	CVec2D Corners[4];

	CRotatedRect() {};
	CRotatedRect(const CVec2D& center, double width, double height, double angle);
};

struct CMyCar {
	CVec2D Position;
	double Angle;
	CRotatedRect RotatedRect;
	CVec2D Speed;
	double AngularSpeed;
	double MedianAngularSpeed;
	double EnginePower;
	double WheelTurn;
	double Durability;
	int NitroCount;
	int NitroTicks;
	int NitroCooldown;
	int OiledTicks;
	int DeadTicks;
	int Type;
	int PlayerId;
	int CollisionsDetected;
	double CollisionDeltaSpeed;

	static const double Width;
	static const double Height;
	static const double HalfWidth;
	static const double HalfHeight;
	static const double CircumcircleRadius;
	static const double CarToWallMomentumTransferFactor;
	static const double CarToWallSurfaceFrictionFactor;
	static const double BaseAngularMass;

	CMyCar();
	CMyCar(const CMyCar& car);
	explicit CMyCar(const model::Car& car);

	double GetMass() const;
	double GetInvertedMass() const;
	double GetAngularMass() const;
	double GetInvertedAngularMass() const;

	void SaveHistory();

	void LogDifference(const CMyCar& car) const;
};
