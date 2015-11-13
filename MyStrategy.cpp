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
		CVec2D prediction = simulator.Predict(self, world, move);

		log.LogCar(self, "self");
		log.LogPosition(prevPosition,              "Previous Position  ");
		log.LogPosition(position,                  "Position           ");
		log.LogPosition(prevPrediction,            "Previous Prediction");
		log.LogPosition(prediction,                "Prediction         ");
		log.LogPosition(position - prevPosition,   "Position shift     ");
		log.LogPosition(position - prevPrediction, "Prediction error   ");

		prevPrediction = prediction;
	}

	prevPosition = position;
}
