#include "MyStrategy.h"

#include <algorithm>
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
	CMyTile tile = CMyTile(currentX, currentY);
	if (tile != currentTile) {
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


	static const int simulationTickDepth = 150;
	CMyTile simTarget = tileRoute[3];
	CVec2D simTargetPos = simTarget.ToVec();

	double bestScore = INT_MIN;
	int bestTurnStart = 0;
	int bestTurnLength = 0;
	int bestTurnReverse = 0;
	int bestTurn = 0;
	int bestBrakeStart = 0;
	int bestBrakeLength = 0;
	int bestTick = INT_MAX;
	CMyCar bestSimCar;
	for (int turn = -1; turn <= 1; turn += 2) {
		for (int brakeStart : {0}) {
			for (int brakeLength : {0, 40} ) {
				for (int turnStart : {0, 40}) {
					//if (turnStart < brakeStart + brakeLength) continue;
					//if (brakeStart < turnStart) continue;
					//brakeStart = max(0, turnStart - 20);
					for (int turnLength : {0, 2, 3, 5, 10, 20, 30, 40, 50, 60, 80, 100}) {
						for (int turnReverse : {turnLength}) {
							const int turnEnd = turnStart + turnLength;
							const int turnReverseAt = turnStart + turnReverse;
							if (turnEnd > simulationTickDepth) continue;
							if (turnReverseAt > turnEnd) continue;
							CMyCar simCar = car;
							for (int tick = 0; tick < simulationTickDepth; tick++) {
								model::Move simMove;
								simMove.setEnginePower(1.0);
								if (tick >= turnStart && tick < turnStart + turnLength) {
									simMove.setWheelTurn(tick >= turnReverseAt ? -turn : turn);
								}
								if (tick >= brakeStart && tick < brakeStart + brakeLength) {
									simMove.setBrake(true);
								}
								simCar = simulator.Predict(simCar, *world, simMove);

								// TODO: ������� score �� ����, ��������� ������ �� ������������ ����� ��������!
								double score = -(simTargetPos - simCar.Position).Length() - 1 * tick;
								if (brakeStart == 0 && brakeLength > 0) score -= 200;
								if (car.Speed.Length() < 15 && brakeStart == 0 && brakeLength > 0) score -= 500;
								//score += (brakeStart == 0 && brakeLength > 0) ? -0.5 : 0;

								if (score > bestScore) {
									bestScore = score;
									bestTurnStart = turnStart;
									bestTurnLength = turnLength;
									bestTurnReverse = turnReverse;
									bestTurn = turn;
									bestTick = tick;
									bestBrakeStart = brakeStart;
									bestBrakeLength = brakeLength;
									bestSimCar = simCar;
								}
							}
						}
					}
				}
			}
		}
	}
	//draw
	CMyCar simCar = car;
	draw.SetColor(0, 0, 255);
	for (int tick = 0; tick < bestTick; tick++) {
		const int turnReverseAt = bestTurnStart + bestTurnReverse;
		model::Move simMove;
		simMove.setEnginePower(1.0);
		if (tick >= bestTurnStart && tick < bestTurnStart + bestTurnLength) {
			simMove.setWheelTurn(tick >= turnReverseAt ? -bestTurn : bestTurn);
		}
		if (tick >= bestBrakeStart && tick < bestBrakeStart + bestBrakeLength) {
			simMove.setBrake(true);
		}
		simCar = simulator.Predict(simCar, *world, simMove);
		draw.FillCircle(simCar.Position, 5);
	}
	const double halfHeight = game->getCarHeight() / 2;
	const double halfWidth = game->getCarWidth() / 2;
	vector<CVec2D> carCorners(4);
	carCorners[0] = CVec2D(halfWidth, halfHeight);
	carCorners[1] = CVec2D(halfWidth, -halfHeight);
	carCorners[2] = CVec2D(-halfWidth, -halfHeight);
	carCorners[3] = CVec2D(-halfWidth, halfHeight);
	draw.SetColor(0, 255, 255);
	draw.FillCircle(simTargetPos, 20);
	for (auto& corner : carCorners) {
		corner.Rotate(simCar.Angle);
		corner += simCar.Position;
		draw.FillCircle(corner, 5);
	}

	resultMove->setEnginePower(1.0);
	if (bestTurnStart == 0 && bestTurnLength > 0) {
		resultMove->setWheelTurn(bestTurn);
	}
	if (bestBrakeStart == 0 && bestBrakeLength > 0) {
		resultMove->setBrake(true);
	}
	// TODO: ����������, ��� ������ �� ���� (�����������?) � ������� ��������� ����� � ����� �����
	// TODO: ���������

	// TODO: ���� ������������ ������?
	//if (self->getNitroChargeCount() > 0) {
	//	resultMove->setUseNitro(true);
	//}
	if (self->getOilCanisterCount() > 0) {
		resultMove->setSpillOil(true);
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
//// ����������� ������� ��� ����� map01 � ����� ������� ������.
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