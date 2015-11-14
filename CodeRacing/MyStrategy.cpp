#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <string>

using namespace model;
using namespace std;

template<typename T>
static const void logIfDiffers(const T& a, const T&b, const char* name, CLog& log)
{
	if (abs(a - b) >= 1e-5) {
		log.Log(a - b, (std::string("Prediction error: ") + name).c_str());
	}
}

MyStrategy::MyStrategy() :
	currentWaypointIndex(0),
	log(CLog::Instance()),
	draw(CDrawPlugin::Instance()),
	currentTick(0)
{
	draw.BeginDraw();
	draw.EndDraw();
}

void MyStrategy::move(const Car& _self, const World& _world, const Game& _game, Move& _resultMove)
{
	self = &_self;
	world = &_world;
	game = &_game;
	resultMove = &_resultMove;

	currentTick = world->getTick();

	firstTick();
	makeMove();
	predict();
	doLog();
	doDraw();

	prevPrediction = prediction;
}

void MyStrategy::firstTick()
{
	if (currentTick == 0) {
		simulator.Initialize(*game);
	}
}

void MyStrategy::makeMove()
{
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		//resultMove->setEnginePower(1.0);
		return;
	}

	// Быстрый старт
	double nextWaypointX = (self->getNextWaypointX() + 0.5) * game->getTrackTileSize();
	double nextWaypointY = (self->getNextWaypointY() + 0.5) * game->getTrackTileSize();

	double cornerTileOffset = 0.25 * game->getTrackTileSize();

	switch (world->getTilesXY()[self->getNextWaypointX()][self->getNextWaypointY()]) {
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

	double angleToWaypoint = self->getAngleTo(nextWaypointX, nextWaypointY);
	double speedModule = hypot(self->getSpeedX(), self->getSpeedY());

	resultMove->setWheelTurn(angleToWaypoint * 32.0 / PI);
	resultMove->setEnginePower(0.75);

	if (speedModule * speedModule * abs(angleToWaypoint) > 2.5 * 2.5 * PI) {
		resultMove->setBrake(true);
	}
}

void MyStrategy::predict()
{
	car = CMyCar(*self);
	prediction = simulator.Predict(car, *world, *resultMove);
}

void MyStrategy::doLog()
{
	log.LogTick(currentTick);
	log.LogMyCar(car,        "Current            ");
	log.LogMyCar(prediction, "Prediction         ");

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
}

void MyStrategy::doDraw()
{
	CDrawPluginSwitcher drawSwitcher(draw);
	draw.SetColor(0, 0, 0);
	draw.FillCircle(car.Position, 10);
	draw.SetColor(255, 128, 0);
	draw.FillCircle(prediction.Position, 5);
	draw.SetColor(0, 0, 255);
	for (int x = 0; x < 10; x++) {
		draw.DrawLine({ x * 800.0, 0.0 }, { x * 800.0, 8000.0 });
	}
	for (int y = 0; y < 10; y++) {
		draw.DrawLine({ 0.0, y * 800.0 }, { 8000.0, y * 800.0 });
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Отладночное руление для карты map01 и места первого игрока.
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
