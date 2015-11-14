#pragma once

#include "model/car.h"
#include "Vec2D.h"

struct CMyCar {
	CVec2D Position;
	CVec2D Speed;
	double Angle;
	double AngularSpeed;
	double EnginePower;
	double WheelTurn;
	int NitroCount;
	int NitroTicks;
	int NitroCooldown;
	int Type;

	CMyCar();
	CMyCar(const CMyCar& car);
	explicit CMyCar(const model::Car& car);

};
