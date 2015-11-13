#pragma once

#include "model/car.h"
#include "Vec2D.h"

struct CMyCar {
	CVec2D Position;
	CVec2D Speed;
	double Angle;
	double AngularSpeed;
	double MedianAngularSpeed;
	double EnginePower;
	double WheelTurn;
	int Type;

	CMyCar();
	CMyCar(const CMyCar& car);
	CMyCar(const model::Car& car, double angularSpeedFactor, const model::Car& previousTickCar);

};
