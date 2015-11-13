#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include "Log.h"

using namespace model;
using namespace std;

MyStrategy::MyStrategy()
{
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	if (!simulator.IsInitialized()) {
		simulator.Initialize(game);
	}
	CLog& log = CLog::Instance();

	if (world.getTick() >= game.getInitialFreezeDurationTicks()) {
		move.setEnginePower(0.9);

		if (world.getTick() == 195) {
			move.setUseNitro(true);
		}

		if (world.getTick() >= 230 && world.getTick() < 300) {
			move.setWheelTurn(1.0);
		} else if (world.getTick() >= 300 && world.getTick() < 341) {
			move.setWheelTurn(-1.0);
		} else {
			move.setWheelTurn(0);
		}

		CMyCar car(self, game.getCarAngularSpeedFactor(), prevSelf);
		CMyCar prediction = simulator.Predict(car, world, move);

		log.LogTick(world.getTick());
		log.LogMyCar(prevPrediction, "Previous prediction");
		log.LogMyCar(car,            "Current            ");
		log.LogMyCar(prediction,     "Prediction         ");
		log.LogVec2D(car.Position - prevPrediction.Position, "Position prediction error");
		log.LogVec2D(car.Speed - prevPrediction.Speed, "Speed prediction error");
		log.Log(car.Angle - prevPrediction.Angle, "Angle prediction error");
		log.Log(car.AngularSpeed - prevPrediction.AngularSpeed, "Angular speed prediction error");

		prevPrediction = prediction;
	}

	prevSelf = self;
}
