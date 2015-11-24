#pragma once

#include "model/car.h"
#include "Vec2D.h"

struct CMyCar {
	struct CRotatedRect {
		CVec2D Corners[4];
	};

	CVec2D Position;
	double Angle;
	CRotatedRect RotatedRect;
	CVec2D Speed;
	double AngularSpeed;
	double EnginePower;
	double WheelTurn;
	int NitroCount;
	int NitroTicks;
	int NitroCooldown;
	int Type;
	bool CollisionDetected;

	CMyCar();
	CMyCar(const CMyCar& car);
	explicit CMyCar(const model::Car& car);

	void UpdateRotatedRect();
};
