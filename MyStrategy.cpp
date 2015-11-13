#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include "Log.h"
#include "Vec2D.h"

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

	log.Log(self, world, game, move);
	move.setEnginePower(1.0);

	if (world.getTick() >= game.getInitialFreezeDurationTicks()) {
		CVec2D nextPosition = simulator.Predict(self, world, move);
	}
}
