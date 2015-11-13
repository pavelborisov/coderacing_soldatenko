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
	CLog::Instance().Log(self, world, game, move);
	move.setEnginePower(1.0);
}
