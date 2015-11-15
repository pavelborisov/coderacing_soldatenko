#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <string>
#include "Tools.h"

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
	log(CLog::Instance()),
	draw(CDrawPlugin::Instance()),
	currentTick(0),
	nextWaypointIndex(0)
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

	CDrawPluginSwitcher drawSwitcher(draw); 
	currentTick = world->getTick();

	firstTick();
	updateWaypoints();
	findTileRoute();
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

void MyStrategy::updateWaypoints()
{
	CMyTile::TileTypesXY = world->getTilesXY();
	CMyTile::TileSize = game->getTrackTileSize();

	vector<vector<int>> waypoints = world->getWaypoints();
	waypointTiles.clear();
	for (const auto& w : waypoints) {
		waypointTiles.push_back(CMyTile(w[0], w[1]));
	}

	while (waypointTiles[nextWaypointIndex].X != self->getNextWaypointX()
		|| waypointTiles[nextWaypointIndex].Y != self->getNextWaypointY())
	{
		nextWaypointIndex++;
		nextWaypointIndex %= waypoints.size();
	}
}

void MyStrategy::findTileRoute()
{
	const int currentX = static_cast<int>(self->getX() / game->getTrackTileSize());
	const int currentY = static_cast<int>(self->getY() / game->getTrackTileSize());
	CMyTile tile = CMyTile(currentX, currentY);
	if (currentTile.IsUndefined()) {
		currentTile = tile;
	} else if (tile == tileRoute[1]) {
		beforePrevTile = prevTile;
		prevTile = currentTile;
		currentTile = tile;
	}
	tileRoute = tileRouteFinder.FindRoute(waypointTiles, nextWaypointIndex, currentTile,
		prevTile, beforePrevTile);
}

void MyStrategy::makeMove()
{
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(1.0);
		return;
	}

	double speedModule = hypot(self->getSpeedX(), self->getSpeedY());

	// TODO: Можно пройтись по всему пути в обратном порядке и проставить инфу о предстоящих поворотах.
	CMyTile nextTile = tileRoute[1];
	CMyTile afterNextTile = tileRoute[2];
	int dx1 = nextTile.X - currentTile.X;
	int dy1 = nextTile.Y - currentTile.Y;
	int angle = getRotationAngle(dx1, dy1);
	simpleRotate(dx1, dy1, angle);
	int dx2 = afterNextTile.X - nextTile.X;
	int dy2 = afterNextTile.Y - nextTile.Y;
	simpleRotate(dx2, dy2, angle);
	bool mirror = dy2 == -1;

	CVec2D targetPos = nextTile.ToVec();
	double angleToTarget = self->getAngleTo(targetPos.X, targetPos.Y);

	if (dy2 != 0) {
		CVec2D cornerShift = CVec2D(-0.25, 0.25);
		if (mirror) cornerShift.Y = -cornerShift.Y;
		simpleRotate(cornerShift.X, cornerShift.Y, 4 - angle);
		targetPos += cornerShift * game->getTrackTileSize();
		angleToTarget = self->getAngleTo(targetPos.X, targetPos.Y);

		if (speedModule > 18) {
			resultMove->setBrake(true);
		}
		if (abs(angleToTarget) > PI / 10 && speedModule > 12) {
			resultMove->setBrake(true);
		}
	}
	resultMove->setWheelTurn(angleToTarget * 30 / PI);

	resultMove->setEnginePower(1.0);
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
	draw.SetColor(0, 0, 0);
	draw.FillCircle(car.Position, 10);
	draw.SetColor(255, 128, 0);
	draw.FillCircle(prediction.Position, 5);

	draw.SetColor(255, 0, 0);
	CVec2D nextWaypoint = waypointTiles[nextWaypointIndex].ToVec();
	draw.FillCircle(nextWaypoint, 50);

	draw.SetColor(0, 255, 0);
	for (size_t i = 1; i < tileRoute.size(); i++) {
		CVec2D from = tileRoute[i - 1].ToVec();
		CVec2D to = tileRoute[i].ToVec();
		draw.DrawLine(from, to);
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
