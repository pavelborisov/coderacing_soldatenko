#include "MyStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <assert.h>
#include "MyWorld.h"
#include "Tools.h"
#include "WaypointsDistanceMap.h"
#include "WorldSimulator.h"
#include <Windows.h>

using namespace model;
using namespace std;

MyStrategy::MyStrategy() :
	log(CLog::Instance()),
	draw(CDrawPlugin::Instance()),
	currentTick(0),
	nextWaypointIndex(0)
{
}

void MyStrategy::move(const Car& _self, const World& _world, const Game& _game, Move& _resultMove)
{
	self = &_self;
	world = &_world;
	game = &_game;
	resultMove = &_resultMove;

	CDrawPluginSwitcher drawSwitcher(draw); 
	currentTick = world->getTick();

	CWorldSimulator::Instance().SetGame(*game);

	currentWorld = CMyWorld(*world, *self);
	currentCar = currentWorld.Cars[0];
	log.LogTick(currentTick);
	log.LogMyCar(currentWorld.Cars[0], "Current            ");

	updateWaypoints();
	findTileRoute();
	makeMove();
	predict();
	doLog();
	doDraw();

	previousPredictedWorld = predictedWorld;
	for (auto& c : predictedWorld.Cars) {
		if (!c.IsValid()) {
			continue;
		}
		c.SaveHistory();
	}
}

void MyStrategy::updateWaypoints()
{
	vector<vector<TileType>> tilesXY = world->getTilesXY();
	bool flush = false;
	if (CMyTile::TileTypesXY.size() == 0) {
		flush = true;
		CMyTile::TileTypesXY = tilesXY;
		CMyTile::TileSize = game->getTrackTileSize();
	}
	for (size_t x = 0; x < tilesXY.size(); x++) {
		for (size_t y = 0; y < tilesXY[0].size(); y++) {
			const model::TileType& src = tilesXY[x][y];
			model::TileType& dst = CMyTile::TileTypesXY[x][y];
			if (src != UNKNOWN && src != dst) {
				flush = true;
				dst = src;
			}
		}
	}

	vector<vector<int>> waypoints = world->getWaypoints();
	waypointTiles.clear();
	for (const auto& w : waypoints) {
		waypointTiles.push_back(CMyTile(w[0], w[1]));
	}

	if (flush) {
		CMyTile::FillWalls();
		CWaypointDistanceMap::Instance().Initialize(waypointTiles);
	}

	nextWaypointIndex = self->getNextWaypointIndex();
}

void MyStrategy::findTileRoute()
{
	const int currentX = static_cast<int>(self->getX() / game->getTrackTileSize());
	const int currentY = static_cast<int>(self->getY() / game->getTrackTileSize());
	currentTile = CMyTile(currentX, currentY);

	int dx = 0;
	int dy = 0;
	if (abs(currentCar.Angle) < PI / 4) {
		dx = 1;
	} else if (abs(currentCar.Angle) > 3 * PI / 4) {
		dx = -1;
	} else if (currentCar.Angle > 0) {
		dy = 1;
	} else if (currentCar.Angle < 0) {
		dy = -1;
	}
	tileRoute = tileRouteFinder.FindRoute(waypointTiles, nextWaypointIndex, currentTile, dx, dy);
}

template<class MAP>
void logStats(const MAP& m, const char* name, std::basic_ostream< char, std::char_traits<char> >& stream)
{
	stream << name << ": ";
	for (auto p : m) {
		stream << p.first << "," << p.second << " ";
	}
	stream << endl;
}

void MyStrategy::makeMove()
{
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(1.0);
		return;
	}

	CBestMoveFinder bestMoveFinder(currentWorld, waypointTiles, previousResult);
	CBestMoveFinder::CResult result = bestMoveFinder.Process();
	previousResult = result;
	*resultMove = result.CurrentMove.Convert();

	// Тупой задний ход
	double angleToTarget = (tileRoute[1].ToVec() - currentCar.Position).GetAngle();
	double angle = angleToTarget - currentCar.Angle;
	normalizeAngle(angle);
	// Когда симулятор хз что делать.
	if (!result.Success || result.MoveList.back().End < 10) {
		CDrawPlugin::Instance().FillCircle(currentCar.Position.X, currentCar.Position.Y, 50, 0x888888);
		resultMove->setEnginePower(1.0);
		resultMove->setWheelTurn(angle * 32 / PI);
	}
	if (rear == 0) {
		if (self->getDurability() == 0) {
			rear = -game->getCarReactivationTimeTicks() - 50;
		} else if (world->getTick() > 200 && currentCar.Speed.Length() < 1) {
			rear = 120 + static_cast<int>(self->getEnginePower() / game->getCarEnginePowerChangePerTick());
		}
	} else if (rear < 0) {
		rear++;
	} else if (rear > 0) {
		resultMove->setBrake(false);
		CDrawPlugin::Instance().FillCircle(currentCar.Position.X, currentCar.Position.Y, 50, 0x880088);
		if (rear < 30) {
			resultMove->setEnginePower(0);
			resultMove->setBrake(true);
			resultMove->setWheelTurn(0);
		} else {
			resultMove->setEnginePower(-1.0);
			resultMove->setWheelTurn(angle > 0 ? -1 : 1);
		}
		rear--;
		if (rear == 0) rear = -120;
	}

	// Тупое нитро.
	if (result.Success) {
		// Сколько тиков поворачиваем.
		int turnTicks = 0;
		int rearTicks = 0;
		int brakeTicks = 0;
		for (const auto& moveWithDuration : result.MoveList) {
			if (moveWithDuration.Move.Turn != 0) {
				turnTicks += moveWithDuration.End - moveWithDuration.Start;
			}
			if (moveWithDuration.Move.Engine < 0) {
				rearTicks += moveWithDuration.End - moveWithDuration.Start;
			}
			if (moveWithDuration.Move.Brake != 0) {
				brakeTicks += moveWithDuration.End - moveWithDuration.Start;
			}
		}
		int totalTicks = result.MoveList.back().End;
		if (self->getNitroChargeCount() > 0 && self->getRemainingNitroCooldownTicks() == 0 && self->getRemainingNitroTicks() == 0
			&& brakeTicks == 0 && rearTicks == 0 && turnTicks <= 15 && totalTicks > 120)
		{
			resultMove->setUseNitro(true);
		}
	}
}

void MyStrategy::predict()
{
	CMyMove moves[4];
	moves[0] = CMyMove(*resultMove);
	moves[1].Engine = 0;
	moves[2].Engine = 0;
	moves[3].Engine = 0;
	CWorldSimulator::Instance().SetPrecision(10);
	CWorldSimulator::Instance().SetOptions(false, false, false);
	predictedWorld = CWorldSimulator::Instance().Simulate(currentWorld, moves);
}

void MyStrategy::doLog()
{
	log.LogMyCar(predictedWorld.Cars[0], "Prediction         ");

	if (currentTick > 180) {
		previousPredictedWorld.LogDifference(currentWorld);
	}
}

void MyStrategy::doDraw()
{
	CVec2D nextWaypoint = waypointTiles[nextWaypointIndex].ToVec();
	draw.FillCircle(nextWaypoint.X, nextWaypoint.Y, 50, 0xFF0000);
}
