#include "MyCar.h"

#include <math.h>

static double MedianAngularSpeedHistory[4] =
{
	UndefinedMedianAngularSpeed,
	UndefinedMedianAngularSpeed,
	UndefinedMedianAngularSpeed, 
	UndefinedMedianAngularSpeed
};

CMyCar::CMyCar() :
	Angle(0),
	AngularSpeed(0),
	MedianAngularSpeed(0),
	EnginePower(0),
	WheelTurn(0),
	NitroCount(0),
	NitroTicks(0),
	NitroCooldown(0),
	OiledTicks(0),
	Type(0),
	Id(0),
	CollisionDetected(false)
{
}

CMyCar::CMyCar(const CMyCar& car) :
	Position(car.Position),
	RotatedRect(car.RotatedRect),
	Angle(car.Angle),
	Speed(car.Speed),
	AngularSpeed(car.AngularSpeed),
	MedianAngularSpeed(car.MedianAngularSpeed),
	EnginePower(car.EnginePower),
	WheelTurn(car.WheelTurn),
	NitroCount(car.NitroCount),
	NitroTicks(car.NitroTicks),
	NitroCooldown(car.NitroCooldown),
	OiledTicks(car.OiledTicks),
	Type(car.Type),
	Id(car.Id),
	CollisionDetected(car.CollisionDetected)
{
}

CMyCar::CMyCar(const model::Car& car) :
	Position(car.getX(), car.getY()),
	Speed(car.getSpeedX(), car.getSpeedY()),
	Angle(car.getAngle()),
	AngularSpeed(car.getAngularSpeed()),
	MedianAngularSpeed(UndefinedMedianAngularSpeed), // »гра не даЄт таких данных
	EnginePower(car.getEnginePower()),
	WheelTurn(car.getWheelTurn()),
	NitroCount(car.getNitroChargeCount()),
	NitroTicks(car.getRemainingNitroTicks()),
	NitroCooldown(car.getRemainingNitroCooldownTicks()),
	OiledTicks(car.getRemainingOiledTicks()),
	Type(car.getType()),
	Id(static_cast<int>(car.getId())),
	CollisionDetected(false)
{
	MedianAngularSpeed = MedianAngularSpeedHistory[Id];
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

void CMyCar::SaveMedianAngularSpeedHistory()
{
	MedianAngularSpeedHistory[Id] = MedianAngularSpeed;
}
