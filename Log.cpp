#include "Log.h"

#ifdef LOGGING

#include "model\Car.h"
#include "model\World.h"
#include "model\Game.h"
#include "model\Move.h"
using namespace std;
using namespace model;

CLog::CLog() : predTick(-1)
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

void CLog::Log(const Car& car, const World& world, const Game& /*game*/, Move& /*move*/)
{
	if (predTick != world.getTick())
	{
		logfile << endl << "Tick: " << world.getTick() << endl;
		predTick = world.getTick();
	}

	logfile << "Car: " << car.getId()
		<< " X: " << car.getX()
		<< " Y: " << car.getY()
		<< " Angle: " << car.getAngle()
		<< " SpeedX: " << car.getSpeedX()
		<< " SpeedY: " << car.getSpeedY()
		<< endl;
}

#endif