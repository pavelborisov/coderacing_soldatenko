#include "MyCar.h"

#include <math.h>

CMyCar::CMyCar() :
	Angle(0),
	AngularSpeed(0),
	EnginePower(0),
	WheelTurn(0),
	NitroCount(0),
	NitroTicks(0),
	NitroCooldown(0),
	Type(0),
	CollisionDetected(false)
{
}

CMyCar::CMyCar(const CMyCar& car) :
	Position(car.Position),
	RotatedRect(car.RotatedRect),
	Angle(car.Angle),
	Speed(car.Speed),
	AngularSpeed(car.AngularSpeed),
	EnginePower(car.EnginePower),
	WheelTurn(car.WheelTurn),
	NitroCount(car.NitroCount),
	NitroTicks(car.NitroTicks),
	NitroCooldown(car.NitroCooldown),
	Type(car.Type),
	CollisionDetected(car.CollisionDetected)
{
}

CMyCar::CMyCar(const model::Car& car) :
	Position(car.getX(), car.getY()),
	Speed(car.getSpeedX(), car.getSpeedY()),
	Angle(car.getAngle()),
	AngularSpeed(car.getAngularSpeed()),
	EnginePower(car.getEnginePower()),
	WheelTurn(car.getWheelTurn()),
	NitroCount(car.getNitroChargeCount()),
	NitroTicks(car.getRemainingNitroTicks()),
	NitroCooldown(car.getRemainingNitroCooldownTicks()),
	Type(car.getType()),
	CollisionDetected(false)
{
	UpdateRotatedRect();
}

void CMyCar::UpdateRotatedRect()
{
	const double halfWidth = 210 / 2;//game.getCarWidth() / 2;
	const double halfHeight = 140 / 2;//game.getCarHeight() / 2;
	for (int i = 0; i < 4; i++) {
		CVec2D& c = RotatedRect.Corners[i];
		c.X = i % 2 == 0 ? halfWidth : -halfWidth;
		c.Y = i / 2 == 0 ? halfHeight : -halfHeight;
		c.Rotate(Angle);
		c += Position;
	}
}
