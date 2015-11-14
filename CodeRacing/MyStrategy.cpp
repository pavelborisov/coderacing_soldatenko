#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <string>
#include "DrawPlugin.h"
#include "Log.h"

using namespace model;
using namespace std;

MyStrategy::MyStrategy()
{
}

template<typename T>
static const void logIfDiffers(const T& a, const T&b, const char* name, CLog& log)
{
	if (abs(a - b) >= 1e-5) {
		log.Log(a - b, (std::string("Prediction error: ") + name).c_str());
	}
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	if (!simulator.IsInitialized()) {
		simulator.Initialize(game);
	}
	CLog& log = CLog::Instance();
	CDrawPlugin& draw = CDrawPlugin::Instance();

	if (world.getTick() >= game.getInitialFreezeDurationTicks()) {
		// Ѕыстрый старт
		double nextWaypointX = (self.getNextWaypointX() + 0.5) * game.getTrackTileSize();
		double nextWaypointY = (self.getNextWaypointY() + 0.5) * game.getTrackTileSize();

		double cornerTileOffset = 0.25 * game.getTrackTileSize();

		switch (world.getTilesXY()[self.getNextWaypointX()][self.getNextWaypointY()]) {
			case LEFT_TOP_CORNER:
				nextWaypointX += cornerTileOffset;
				nextWaypointY += cornerTileOffset;
				break;
			case RIGHT_TOP_CORNER:
				nextWaypointX -= cornerTileOffset;
				nextWaypointY += cornerTileOffset;
				break;
			case LEFT_BOTTOM_CORNER:
				nextWaypointX += cornerTileOffset;
				nextWaypointY -= cornerTileOffset;
				break;
			case RIGHT_BOTTOM_CORNER:
				nextWaypointX -= cornerTileOffset;
				nextWaypointY -= cornerTileOffset;
				break;
		}

		double angleToWaypoint = self.getAngleTo(nextWaypointX, nextWaypointY);
		double speedModule = hypot(self.getSpeedX(), self.getSpeedY());

		move.setWheelTurn(angleToWaypoint * 32.0 / PI);
		move.setEnginePower(0.75);

		if (speedModule * speedModule * abs(angleToWaypoint) > 2.5 * 2.5 * PI) {
			move.setBrake(true);
		}

		// —имул€ци€
		CMyCar car(self);
		CMyCar prediction = simulator.Predict(car, world, move);

		log.LogTick(world.getTick());
		//log.LogMyCar(prevPrediction, "Previous prediction");
		log.LogMyCar(car,            "Current            ");
		log.LogMyCar(prediction,     "Prediction         ");

		logIfDiffers(car.Angle, prevPrediction.Angle, "Angle", log);
		logIfDiffers(car.AngularSpeed, prevPrediction.AngularSpeed, "AngularSpeed", log);
		logIfDiffers(car.EnginePower, prevPrediction.EnginePower, "EnginePower", log);
		logIfDiffers(car.NitroCooldown, prevPrediction.NitroCooldown, "NitroCooldown", log);
		logIfDiffers(car.NitroCount, prevPrediction.NitroCount, "NitroCount", log);
		logIfDiffers(car.NitroTicks, prevPrediction.NitroTicks, "NitroTicks", log);
		logIfDiffers(car.Position.X, prevPrediction.Position.X, "Position.X", log);
		logIfDiffers(car.Position.Y, prevPrediction.Position.Y, "Position.Y", log);
		logIfDiffers(car.Speed.X, prevPrediction.Speed.X, "Speed.X", log);
		logIfDiffers(car.Speed.Y, prevPrediction.Speed.Y, "Speed.Y", log);
		logIfDiffers(car.Type, prevPrediction.Type, "Type", log);

		CDrawPluginSwitcher drawSwitcher(draw);
		draw.FillCircle(car.Position, 10);
		draw.FillCircle(prediction.Position, 5);

		prevPrediction = prediction;
	}

	prevSelf = self;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// ќтладночное руление дл€ карты map01 и места первого игрока.
//if (world.getTick() >= game.getInitialFreezeDurationTicks()) {
//	move.setEnginePower(0.9);
//
//	if (world.getTick() == 195) {
//		move.setUseNitro(true);
//	}
//
//	if (world.getTick() >= 230 && world.getTick() < 300) {
//		move.setWheelTurn(1.0);
//	} else if (world.getTick() >= 300 && world.getTick() < 341) {
//		move.setWheelTurn(-1.0);
//	}
//
//	if (world.getTick() >= 450 && world.getTick() <= 600) {
//		move.setBrake(true);
//	}
//}
