#include "Log.h"

#ifdef LOGGING

#include "model\Car.h"
#include "model\World.h"
#include "model\Game.h"
#include "model\Move.h"
#include "Vec2D.h"
using namespace std;
using namespace model;

CLog::CLog()
{
	const string logpath = "..\\log.txt";

	logfile.open(logpath.c_str());

	logfile.setf(ios::fixed, ios::floatfield);
	logfile.precision(3);
}

CLog::~CLog()
{
	logfile.close();
}

void CLog::LogTick(int tick)
{
	logfile << endl << "Tick: " << tick << endl;
}

void CLog::LogCar(const Car& car, const char* name)
{
	logfile << name << ":"
		<< " Id: " << car.getId()
		<< " X: " << car.getX()
		<< " Y: " << car.getY()
		<< " Angle: " << car.getAngle()
		<< " SpeedX: " << car.getSpeedX()
		<< " SpeedY: " << car.getSpeedY()
		<< endl;
}

void CLog::LogPosition(const CVec2D& position, const char* name)
{
	logfile << name << ":"
		<< " X: " << position.X
		<< " Y: " << position.Y
		<< endl;

}

#endif