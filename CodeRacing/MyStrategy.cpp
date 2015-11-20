#include "MyStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <assert.h>
#include "BestMoveFinder.h"
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

	car = CMyCar(*self);
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
	vector<vector<TileType>> tilesXY = world->getTilesXY();
	if (CMyTile::TileTypesXY.size() == 0) {
		CMyTile::TileTypesXY = tilesXY;
		CMyTile::TileSize = game->getTrackTileSize();
	}
	for (size_t x = 0; x < tilesXY.size(); x++) {
		for (size_t y = 0; y < tilesXY.size(); y++) {
			if (tilesXY[x][y] != UNKNOWN) {
				CMyTile::TileTypesXY[x][y] = tilesXY[x][y];
			}
		}
	}

	vector<vector<int>> waypoints = world->getWaypoints();
	waypointTiles.clear();
	for (const auto& w : waypoints) {
		waypointTiles.push_back(CMyTile(w[0], w[1]));
	}

	nextWaypointIndex = self->getNextWaypointIndex();
}

void MyStrategy::findTileRoute()
{
	const int currentX = static_cast<int>(self->getX() / game->getTrackTileSize());
	const int currentY = static_cast<int>(self->getY() / game->getTrackTileSize());
	currentTile = CMyTile(currentX, currentY);

	// TODO: Сделать поиск маршрута в зависимости от направления скорости машины в начальной точке.
	tileRoute = tileRouteFinder.FindRoute(waypointTiles, nextWaypointIndex, currentTile);
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

// TODO: Вынести весь перебор в отдельный файл
void MyStrategy::makeMove()
{
	// TODO: Баг с предсказанием торможения в повороте.
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(1.0);
		return;
	}

	CBestMoveFinder bestMoveFinder(car, *world, *game, tileRoute, simulator);
	CBestMoveFinder::CResult result = bestMoveFinder.Process();
	*resultMove = result.CurrentMove.Convert();

	// Тупой задний ход
	// TODO: Запомнить, куда мы поворачивали колеса в последний раз и задним ходом делать наборот.
	static int rear = 0;
	if (rear == 0) {
		if (self->getDurability() == 0) {
			rear = -game->getCarReactivationTimeTicks() - 50;
		} else if (world->getTick() > 200 && car.Speed.Length() < 2) {
			rear = 150;
		}
	} else if (rear < 0) {
		rear++;
	} else if (rear > 0) {
		double angle = car.Angle - (car.Position - tileRoute[1].ToVec()).GetAngle();
		normalizeAngle(angle);

		if (rear < 30) {
			resultMove->setBrake(true);
			resultMove->setEnginePower(1.0);
			resultMove->setWheelTurn(angle);
		} else {
			resultMove->setBrake(false);
			resultMove->setEnginePower(-1.0);
			resultMove->setWheelTurn(-angle);
		}
		rear--;
		if (rear == 0) rear = -150;
	}

	for (auto otherCar : world->getCars()) {
		if (otherCar.getPlayerId() != self->getPlayerId()) {
			double angle = self->getAngleTo(otherCar);
			double dist = self->getDistanceTo(otherCar);
			// Тупая стрелялка
			if (self->getProjectileCount() > 0
				&& (abs(angle) < PI / 90 && dist < 1500)
					||(abs(angle) < PI/30 && dist < 700)
					||(abs(angle) < PI/9 && dist < 300 ))
			{
					resultMove->setThrowProjectile(true);
			}
			// Тупая лужа
			if (world->getTick() > 350 && self->getOilCanisterCount() > 0 && abs(self->getWheelTurn()) > 0.5) {
				resultMove->setSpillOil(true);
			}
		}
	}
	// TODO: Проверка на прямые участки.
	// Тупое нитро.
	// Сколько тиков поворачиваем.
	int turnTicks = 0;
	for (const auto& moveWithDuration : result.MoveList) {
		if (moveWithDuration.Move.Turn != 0) {
			turnTicks += moveWithDuration.End - moveWithDuration.Start;
		}
	}
	int totalTicks = result.MoveList.back().End;
	if (self->getNitroChargeCount() > 0 && self->getRemainingNitroCooldownTicks() == 0 && self->getRemainingNitroTicks() == 0
		&& turnTicks < 20 && totalTicks > 120)
	{
		resultMove->setUseNitro(true);
	}
}

void MyStrategy::predict()
{
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
	for (size_t i = 1; i < min(10U, tileRoute.size()); i++) {
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
