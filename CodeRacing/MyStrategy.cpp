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

	if (world->getPlayers().size() == 2 && self->getType() == JEEP) return;

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
	experiment();
	predict();
	doLog();
	doDraw();

	previousPredictedWorld = predictedWorld;
	for (auto& c : predictedWorld.Cars) {
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
	processShooting();
	processOil();

	// Тупой задний ход
	static int rear = 0;
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

void MyStrategy::predictObjects()
{
	//predictions[0] = currentWorld;
	//CWorldSimulator::Instance().SetPrecision(2);
	//CWorldSimulator::Instance().SetOptions(false, false, false);
	//CMyMove moves[4];
	//for (int tick = 1; tick < predictionsSize; tick++) {
	//	for (const auto& m : previousResult.MoveList) {
	//		if (tick >= m.Start && tick < m.End) {
	//			moves[0] = m.Move;
	//		}
	//	}
	//	predictions[tick] = CWorldSimulator::Instance().Simulate(predictions[tick - 1], moves);
	//}
}

void MyStrategy::processShooting()
{
	//CMyWorld shootingPredictions[predictionsSize];
	//shootingPredictions[0] = currentWorld;
	//if (currentCar.Type == 0) {
	//	auto& washers = shootingPredictions[0].Washers;
	//	int i = 0;
	//	while (i < CMyWorld::MaxWashers && washers[i].IsValid()) i++;
	//	for (int j = -1; j <= 1; j++) {
	//		if (i < CMyWorld::MaxWashers) {
	//			washers[i].CarId = 0;
	//			washers[i].Position = currentCar.Position;
	//			const double angle = currentCar.Angle + j * game->getSideWasherAngle();
	//			washers[i].Speed = CVec2D(cos(angle), sin(angle)) * game->getWasherInitialSpeed();
	//			i++;
	//		}
	//	}
	//} else {
	//	auto& tires = shootingPredictions[0].Tires;
	//	int i = 0;
	//	while (i < CMyWorld::MaxTires && tires[i].IsValid()) i++;
	//	if (i < CMyWorld::MaxTires) {
	//		tires[i].CarId = 0;
	//		tires[i].Position = currentCar.Position;
	//		tires[i].Speed = CVec2D(cos(currentCar.Angle), sin(currentCar.Angle)) * game->getTireInitialSpeed();
	//		tires[i].AngularSpeed = 0;
	//		i++;
	//	}
	//}

	//CWorldSimulator::Instance().SetPrecision(2);
	//CWorldSimulator::Instance().SetOptions(false, false, false);
	//CMyMove moves[4];
	//for (int tick = 1; tick < predictionsSize; tick++) {
	//	for (const auto& m : previousResult.MoveList) {
	//		if (tick >= m.Start && tick < m.End) {
	//			moves[0] = m.Move;
	//		}
	//	}
	//	shootingPredictions[tick] = CWorldSimulator::Instance().Simulate(shootingPredictions[tick - 1], moves);
	//}

}

void MyStrategy::processOil()
{
	//if (self->getOilCanisterCount() == 0) {
	//	return;
	//}

	//static const int predictionLength = 50;
	//static const double minEnemySpeed = 5;
	//static const double minEnemySpeed2 = 20;
	//static const double minEnemyAngularSpeed = PI / 90;
	//static const double minEnemyDrift = PI / 20;
	//double speedRelAngle = car.Speed.GetAngle() - car.Angle;
	//normalizeAngle(speedRelAngle);
	//if (abs(speedRelAngle) > PI / 2) {
	//	// Мы сбросим лужу себе под колёса.
	//	return;
	//}

	//const double oilRadius = game->getOilSlickRadius();
	//CVec2D oilOffset(game->getOilSlickInitialRange() + game->getCarWidth() / 2 + oilRadius, 0);
	//oilOffset.Rotate(car.Angle);
	//const CVec2D oilPos = car.Position - oilOffset;
	////CDrawPlugin::Instance().SetColor(196, 196, 196);
	////CDrawPlugin::Instance().FillCircle(oilPos, oilRadius);

	//// Выбрасываем лужу только, когда мы уверены, что в неё попадёт какой-либо враг сзади.
	//// Попадание в мазут считается если центр машины попал внутрь окружности мазута.
	//const double oilRadiusSqr = pow(oilRadius, 2);
	//for (size_t enemyIndex = 0; enemyIndex < CGlobalPredictions::EnemyCarsPerTick.size(); enemyIndex++) {
	//	for (int tick = 1; tick <= predictionLength; tick++) {
	//		const CMyCar& enemy = CGlobalPredictions::EnemyCarsPerTick[enemyIndex][tick];
	//		// Проверка, что автомобиль попал.
	//		if ((enemy.Position - oilPos).LengthSquared() < oilRadiusSqr) {
	//			double enemySpeedRelAngle = enemy.Speed.GetAngle() - enemy.Angle;
	//			normalizeAngle(enemySpeedRelAngle);
	//			double enemyDrift = abs(enemySpeedRelAngle);
	//			enemyDrift = enemyDrift > PI / 2 ? PI - enemyDrift : enemyDrift;
	//			if (enemy.Speed.Length() > minEnemySpeed && 
	//				(enemyDrift > minEnemyDrift || abs(enemy.AngularSpeed > minEnemyAngularSpeed)))
	//			{
	//				resultMove->setSpillOil(true);
	//				return;
	//			}
	//			if (enemy.Speed.Length() > minEnemySpeed2) {
	//				resultMove->setSpillOil(true);
	//				return;
	//			}
	//		}
	//	}
	//}
}
void MyStrategy::experiment()
{
	//double dist = CWaypointDistanceMap::Instance().Query(car.Position.X, car.Position.Y, car.Angle, nextWaypointIndex);
	//log.Stream() << dist;
	//for (int tileX = 0; tileX < CMyTile::SizeX(); tileX++) {
	//	const int startX = tileX * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//	const int endX = (tileX + 1) * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//	for (int tileY = 0; tileY < CMyTile::SizeY(); tileY++) {
	//		const int startY = tileY * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//		const int endY = (tileY + 1) * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//		for (int x = startX; x < endX; x++) {
	//			for (int y = startY; y < endY; y++) {
	//				const double worldX = (x + 0.5) * CWaypointDistanceMap::step;
	//				const double worldY = (y + 0.5) * CWaypointDistanceMap::step;
	//				//dist = CWaypointDistanceMap::Instance().QueryBestDirection(worldX, worldY, nextWaypointIndex);
	//				dist = CWaypointDistanceMap::Instance().Query(worldX, worldY, car.Angle, nextWaypointIndex);
	//				//dist;
	//				CDrawPlugin::Instance().Text(worldX, worldY, to_string((int)dist).c_str(), 0x000000);
	//			}
	//		}
	//	}
	//}
	//for (const auto& t : CGlobalPredictions::TiresPerTick) {
	//	for (int i = 0; i < 3; i++) {
	//		CLog::Instance().Stream() << "Tire Tick " << i << ": "
	//			<< t[i].Position.X << "," << t[i].Position.Y << " "
	//			<< t[i].Speed.X << "," << t[i].Speed.Y << " "
	//			<< t[i].AngularSpeed << endl;
	//	}
	//	for (int i = 0; i < 30; i++) {
	//		CDrawPlugin::Instance().Circle(t[i].Position.X, t[i].Position.Y, CMyTire::Radius, 0xFF0000);
	//	}
	//}

	//CMyWorld simWorld = myWorld;
	//CWorldSimulator::Instance().SetPrecision(2);
	//for (int i = 0; i < 100; i++) {
	//	simWorld = CWorldSimulator::Instance().Simulate(simWorld, moves);
	//	int green = 100 + i;
	//	simWorld.Draw(0xFF00FF + 0x000100 * green);
	//}
	//CWorldSimulator::Instance().SetPrecision(10);
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

	//for (const auto& c : prediction.RotatedRect.Corners) {
	//	draw.FillCircle(c.X, c.Y, 1, 0x005500);
	//}

	//for (size_t i = 1; i < min(10U, tileRoute.size()); i++) {
	//	CVec2D from = tileRoute[i - 1].ToVec();
	//	CVec2D to = tileRoute[i].ToVec();
	//	draw.Line(from.X, from.Y, to.X, to.Y, 0x00FF00);
	//}
}
