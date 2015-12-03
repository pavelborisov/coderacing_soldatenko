#pragma once

#include "model/car.h"
#include "Vec2D.h"

const double UndefinedMedianAngularSpeed = -123456;

struct CMyCar {
	struct CRotatedRect {
		CVec2D Corners[4];
	};

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
	bool CollisionDetected;

	CMyCar();
	CMyCar(const CMyCar& car);
	explicit CMyCar(const model::Car& car);

	void UpdateRotatedRect();
	void SaveHistory();
};
