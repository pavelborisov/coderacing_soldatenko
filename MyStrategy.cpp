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
	log.LogTick(world.getTick());
	CVec2D position = CVec2D(self.getX(), self.getY());

	move.setEnginePower(1.0);

	if (world.getTick() >= game.getInitialFreezeDurationTicks()) {
		CMyCar car(self);
		CMyCar prediction = simulator.Predict(car, world, move);

		log.LogMyCar(car,            "Current            ");
		log.LogMyCar(prevPrediction, "Previous prediction");
		log.LogMyCar(prediction,     "Prediction         ");
		log.LogVec2D(car.Position - prevPrediction.Position, "Position prediction error");

		prevPrediction = prediction;
	}

	prevSelf = self;
}
