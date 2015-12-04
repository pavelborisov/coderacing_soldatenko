#include "MyCar.h"

#include <math.h>
#include <assert.h>

static double MedianAngularSpeedHistory[10] =
{
	UndefinedMedianAngularSpeed, UndefinedMedianAngularSpeed,
	UndefinedMedianAngularSpeed, UndefinedMedianAngularSpeed,
	UndefinedMedianAngularSpeed, UndefinedMedianAngularSpeed,
	UndefinedMedianAngularSpeed, UndefinedMedianAngularSpeed,
	UndefinedMedianAngularSpeed, UndefinedMedianAngularSpeed,
};

static int DeadTicksHistory[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static int HistoryId(int PlayerId, int Type)
{
	assert(PlayerId >= 1 && PlayerId <= 4 && (Type == 0 || Type == 1));
	const int id = PlayerId * 2 + Type;
	assert(id >= 0 && id < 10);
	return id;
}

CRotatedRect::CRotatedRect(const CVec2D& center, double width, double height, double angle)
{
	const double halfWidth = width / 2;
	const double halfHeight = height / 2;
	for (int i = 0; i < 4; i++) {
		CVec2D& c = Corners[i];
		c.X = i % 2 == 0 ? halfWidth : -halfWidth;
		c.Y = i / 2 == 0 ? halfHeight : -halfHeight;
		c.Rotate(angle);
		c += center;
	}
}

CMyCar::CMyCar() :
	Angle(0),
	AngularSpeed(0),
	MedianAngularSpeed(0),
	EnginePower(0),
	WheelTurn(0),
	Durability(0),
	NitroCount(0),
	NitroTicks(0),
	NitroCooldown(0),
	OiledTicks(0),
	DeadTicks(0),
	Type(0),
	PlayerId(0),
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
	Durability(car.Durability),
	NitroCount(car.NitroCount),
	NitroTicks(car.NitroTicks),
	NitroCooldown(car.NitroCooldown),
	OiledTicks(car.OiledTicks),
	DeadTicks(car.DeadTicks),
	Type(car.Type),
	PlayerId(car.PlayerId),
	CollisionDetected(car.CollisionDetected)
{
}

CMyCar::CMyCar(const model::Car& car) :
	Position(car.getX(), car.getY()),
	Speed(car.getSpeedX(), car.getSpeedY()),
	Angle(car.getAngle()),
	AngularSpeed(car.getAngularSpeed()),
	MedianAngularSpeed(UndefinedMedianAngularSpeed), // Игра не даёт таких данных
	EnginePower(car.getEnginePower()),
	WheelTurn(car.getWheelTurn()),
	Durability(car.getDurability()),
	NitroCount(car.getNitroChargeCount()),
	NitroTicks(car.getRemainingNitroTicks()),
	NitroCooldown(car.getRemainingNitroCooldownTicks()),
	OiledTicks(car.getRemainingOiledTicks()),
	DeadTicks(0), // Как узнать, сколько тиков ещё машина будет дохлой?
	Type(car.getType()),
	PlayerId(static_cast<int>(car.getPlayerId())),
	CollisionDetected(false)
{
	MedianAngularSpeed = MedianAngularSpeedHistory[HistoryId(PlayerId, Type)];
	DeadTicks = Durability > 1e-5 ? 0 : DeadTicksHistory[HistoryId(PlayerId, Type)];
	RotatedRect = CRotatedRect(Position, 210, 140, Angle);
}

void CMyCar::SaveHistory()
{
	MedianAngularSpeedHistory[HistoryId(PlayerId, Type)] = MedianAngularSpeed;
	DeadTicksHistory[HistoryId(PlayerId, Type)] = DeadTicks;
}
