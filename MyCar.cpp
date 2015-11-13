#include "MyCar.h"

CMyCar::CMyCar() :
	Angle(0),
	AngularSpeed(0),
	EnginePower(0),
	WheelTurn(0)
{
}

CMyCar::CMyCar(const CMyCar& car) :
	Position(car.Position),
	Speed(car.Speed),
	Angle(car.Angle),
	AngularSpeed(car.AngularSpeed),
	EnginePower(car.EnginePower),
	WheelTurn(car.WheelTurn)
{
}

CMyCar::CMyCar(const model::Car& car) :
	Position(car.getX(), car.getY()),
	Speed(car.getSpeedX(), car.getSpeedY()),
	Angle(car.getAngle()),
	AngularSpeed(car.getAngularSpeed()),
	EnginePower(car.getEnginePower()),
	WheelTurn(car.getWheelTurn())
{
}
