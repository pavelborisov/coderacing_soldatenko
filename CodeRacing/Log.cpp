#include "Log.h"

#ifdef LOGGING

#include "model/World.h"
#include "model/Game.h"
#include "model/Move.h"
#include "MyCar.h"
#include "Vec2D.h"
using namespace std;
using namespace model;

CLog::CLog()
{
	const string logpath = "..\\log.txt";

	logfile.open(logpath.c_str());

	logfile.setf(ios::fixed, ios::floatfield);
	logfile.precision(5);
}

CLog::~CLog()
{
	logfile.close();
}

void CLog::LogTick(int tick)
{
	logfile << endl << "Tick: " << tick << endl;
}

void CLog::LogMyCar(const CMyCar& car, const char* name)
{
	logfile << name << ":"
		<< " X: " << car.Position.X
		<< " Y: " << car.Position.Y
		<< " SpeedX: " << car.Speed.X
		<< " SpeedY: " << car.Speed.Y
		<< " Angle: " << car.Angle
		<< " AngularSpeed: " << car.AngularSpeed
		<< " EnginePower: " << car.EnginePower
		<< " WheelTurn: " << car.WheelTurn
		<< " NitroCount:" << car.NitroCount
		<< " NitroTicks:" << car.NitroTicks
		<< " NitroCooldown: " << car.NitroCooldown
		<< " OiledTicks: " << car.OiledTicks
		<< " Type:" << car.Type
		<< " Id:" << car.Id
		<< endl;
}

void CLog::LogVec2D(const CVec2D& position, const char* name)
{
	logfile << name << ":"
		<< " X: " << position.X
		<< " Y: " << position.Y
		<< endl;
}

#endif