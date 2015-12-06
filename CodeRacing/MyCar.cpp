#include "MyCar.h"

#include <math.h>
#include <assert.h>
#include "Log.h"

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

const double CMyCar::Width = 210;
const double CMyCar::Height = 140;
const double CMyCar::HalfWidth = Width / 2;
const double CMyCar::HalfHeight = Height / 2;
const double CMyCar::CircumcircleRadius = sqrt(pow(HalfWidth, 2) + pow(HalfHeight, 2));
const double CMyCar::CarToWallMomentumTransferFactor = 0.25;
const double CMyCar::CarToWallSurfaceFrictionFactor = 0.25 * 0.25;
const double CMyCar::BaseAngularMass = 1.0 / 12 * (Width * Width + Height * Height);

CRotatedRect::CRotatedRect(const CVec2D& center, double width, double height, double angle)
{
	const double halfWidth = width / 2;
	const double halfHeight = height / 2;
	for (int i = 0; i < 4; i++) {
		CVec2D& c = Corners[i];
		c.X = (i == 0 || i == 3) ? halfWidth : -halfWidth;
		c.Y = (i == 0 || i == 1) ? halfHeight : -halfHeight;
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
	CollisionsDetected(0),
	CollisionDeltaSpeed(0)
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
	CollisionsDetected(car.CollisionsDetected),
	CollisionDeltaSpeed(car.CollisionDeltaSpeed)
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
	CollisionsDetected(false),
	CollisionDeltaSpeed(0)
{
	MedianAngularSpeed = MedianAngularSpeedHistory[HistoryId(PlayerId, Type)];
	DeadTicks = Durability > 1e-5 ? 0 : DeadTicksHistory[HistoryId(PlayerId, Type)];
	RotatedRect = CRotatedRect(Position, 210, 140, Angle);
}

double CMyCar::GetMass() const
{
	return Type == 0 ? 1250 : 1500;
}

double CMyCar::GetInvertedMass() const
{
	return Type == 0 ? 1.0 / 1250 : 1.0 / 1500;
}

double CMyCar::GetAngularMass() const
{
	return (Type == 0 ? 1250 : 1500) * BaseAngularMass;
}

double CMyCar::GetInvertedAngularMass() const
{
	return 1.0 / ((Type == 0 ? 1250 : 1500) * BaseAngularMass);
}

void CMyCar::SaveHistory()
{
	MedianAngularSpeedHistory[HistoryId(PlayerId, Type)] = MedianAngularSpeed;
	DeadTicksHistory[HistoryId(PlayerId, Type)] = DeadTicks;
}

template<class T>
static const void logIfDifferent(const T& a, const T& b, const char* name)
{
	if (abs(a - b) > 1e-5) {
		CLog::Instance().Stream() << "Warning! Different " << name << ": "
			<< a << " " << b << " " << a - b << std::endl;
	}
}

void CMyCar::LogDifference(const CMyCar& car) const
{
#ifdef LOGGING
	logIfDifferent(Position.X, car.Position.X, "Position.X");
	logIfDifferent(Position.Y, car.Position.Y, "Position.Y");
	logIfDifferent(Angle, car.Angle, "Angle");
	//CRotatedRect RotatedRect;
	logIfDifferent(Speed.X, car.Speed.X, "Speed.X");
	logIfDifferent(Speed.Y, car.Speed.Y, "Speed.Y");
	logIfDifferent(AngularSpeed, car.AngularSpeed, "AngularSpeed");
	logIfDifferent(MedianAngularSpeed, car.MedianAngularSpeed, "MedianAngularSpeed");
	logIfDifferent(EnginePower, car.EnginePower, "EnginePower");
	logIfDifferent(WheelTurn, car.WheelTurn, "WheelTurn");
	logIfDifferent(Durability, car.Durability, "Durability");
	logIfDifferent(NitroCount, car.NitroCount, "NitroCount");
	logIfDifferent(NitroTicks, car.NitroTicks, "NitroTicks");
	logIfDifferent(NitroCooldown, car.NitroCooldown, "NitroCooldown"); 
	logIfDifferent(OiledTicks, car.OiledTicks, "OiledTicks");
	logIfDifferent(DeadTicks, car.DeadTicks, "DeadTicks");
	logIfDifferent(Type, car.Type, "Type");
	logIfDifferent(PlayerId, car.PlayerId, "Type");
	//int CollisionsDetected;
	//double CollisionDeltaSpeed;
#endif
}
