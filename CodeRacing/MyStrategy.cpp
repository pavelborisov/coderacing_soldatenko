#include "MyStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <assert.h>
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

void MyStrategy::makeMove()
{
	// TODO: Баг с предсказанием торможения в повороте.
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(1.0);
		return;
	}

	static const int simulationDepth = 175;

	double bestScore = INT_MIN;
	CMyCar bestSimCar;
	int bestTick = INT_MAX;
	int bestFirstAction = 0;
	int bestFirstLength = 0;
	int bestSecondAction = 0;
	int bestSecondLength = 0;
	int bestThirdAction = 0;
	int bestThirdLength = 0;

	static map<int, int> bestTickStats;
	static map<int, int> bestFirstActionStats;
	static map<int, int> bestFirstLengthStats;
	static map<int, int> bestSecondActionStats;
	static map<int, int> bestSecondLengthStats;
	static map<int, int> bestThirdActionStats;
	static map<int, int> bestThirdLengthStats;

	vector<int> firstActionArray = { 0, -1, 1 };
	vector<int> firstLengthArray = { 0, 5, 10, 20, 30 };
	vector<int> secondActionArray = { 0, INT_MIN };
	vector<int> secondLengthArray = { 0, 30 };
	vector<int> thirdActionArray = { -1, 1 };
	vector<int> thirdLengthArray = { 0, 10, 25, 50 };

	for(int firstAction: firstActionArray) {
		for (int firstLength : firstLengthArray) {
			int firstStart = 0;
			int firstEnd = firstStart + firstLength;
			if (firstLength == 0 && firstAction != firstActionArray[0]) continue;
			if (firstEnd > simulationDepth) continue;
			for (int secondAction : secondActionArray) {
				for (int secondLength : secondLengthArray) {
					int secondStart = firstEnd;
					int secondEnd = secondStart + secondLength;
					if (secondLength == 0 && secondAction != secondActionArray[0]) continue;
					if (secondEnd > simulationDepth) continue;
					for (int thirdAction : thirdActionArray) {
						for (int thirdLength : thirdLengthArray) {
							int thirdStart = secondEnd;
							int thirdEnd = thirdStart + thirdLength;
							if (thirdLength == 0 && thirdAction != thirdActionArray[0]) continue;
							if (thirdEnd > simulationDepth) continue;

							//////////// непосредственно симуляция.
							CMyCar simCar = car;
							int nextRouteIndex = 1;
							double routeScore = 0;
							for (int tick = 0; tick < simulationDepth; tick++) {
								model::Move simMove;
								simMove.setEnginePower(1.0);
								if (tick >= firstStart && tick < firstEnd) {
									if (firstAction == INT_MIN) {
										simMove.setBrake(true);
									} else {
										simMove.setWheelTurn(firstAction);
									}
								} else if (tick >= secondStart && tick < secondEnd) {
									if (secondAction == INT_MIN) {
										simMove.setBrake(true);
									} else {
										simMove.setWheelTurn(secondAction);
									}
								} else if (tick >= thirdStart && tick < thirdEnd) {
									if (thirdAction == INT_MIN) {
										simMove.setBrake(true);
									} else {
										simMove.setWheelTurn(thirdAction);
									}
								}
								simCar = simulator.Predict(simCar, *world, simMove);

								CMyTile simCarTile(simCar.Position);
								double score = 0;
								if (simCarTile == tileRoute[nextRouteIndex]) {
									nextRouteIndex = (nextRouteIndex + 1) % tileRoute.size();
									routeScore += 800;
								}
								score += routeScore;
								CMyTile targetTile = tileRoute[nextRouteIndex];
								// Проверяем из какого тайл в какой едем. Увеличиваем оценку на то, сколько мы проехали вдоль направления,
								// по которому эти тайлы соединяются.
								if (simCarTile.X == targetTile.X + 1) {
									score += (simCarTile.X + 1) * 800 - simCar.Position.X;
								} else if (simCarTile.X == targetTile.X - 1) {
									score += simCar.Position.X - (simCarTile.X) * 800;
								} else if (simCarTile.Y == targetTile.Y + 1) {
									score += (simCarTile.Y + 1) * 800 - simCar.Position.Y;
								} else if (simCarTile.Y == targetTile.Y - 1) {
									score += simCar.Position.Y - (simCarTile.Y) * 800;
								} else {
									// Где-то далеко мы находимся.
									score -= 0;
								}

								// TODO: штраф за напрасное торможение?
								//if (brakeStart == 0 && brakeLength > 0) score -= 200;

								if (score > bestScore) {
									bestScore = score;
									bestSimCar = simCar;
									bestTick = tick;
									bestFirstAction = firstAction;
									bestFirstLength = firstLength;
									bestSecondAction = secondAction;
									bestSecondLength = secondLength;
									bestThirdAction = thirdAction;
									bestThirdLength = thirdLength;
								}

								if (simCar.CollisionDetected) {
									break;
								}
							}
							//////////// конец непосредственно симуляции.
						}
					}
				}
			}
		}
	}

	////////////////////////////////////////// draw best
	//CMyCar simCar = car;
	//draw.SetColor(0, 0, 255);
	//for (int tick = 0; tick < bestTick; tick++) {
	//	model::Move simMove;
	//	simMove.setEnginePower(1.0);
	//	int firstStart = 0;
	//	int firstEnd = firstStart + bestFirstLength;
	//	int secondStart = firstEnd;
	//	int secondEnd = secondStart + bestSecondLength;
	//	int thirdStart = secondEnd;
	//	int thirdEnd = thirdStart + bestThirdLength;

	//	if (tick >= firstStart && tick < firstEnd) {
	//		if (bestFirstAction == INT_MIN) {
	//			simMove.setBrake(true);
	//		} else {
	//			simMove.setWheelTurn(bestFirstAction);
	//		}
	//	} else if (tick >= secondStart && tick < secondEnd) {
	//		if (bestSecondAction == INT_MIN) {
	//			simMove.setBrake(true);
	//		} else {
	//			simMove.setWheelTurn(bestSecondAction);
	//		}
	//	} else if (tick >= thirdStart && tick < thirdEnd) {
	//		if (bestThirdAction == INT_MIN) {
	//			simMove.setBrake(true);
	//		} else {
	//			simMove.setWheelTurn(bestThirdAction);
	//		}
	//	}
	//	simCar = simulator.Predict(simCar, *world, simMove);
	//	draw.FillCircle(simCar.Position, 5);
	//}
	//const double halfHeight = game->getCarHeight() / 2;
	//const double halfWidth = game->getCarWidth() / 2;
	//vector<CVec2D> carCorners(4);
	//carCorners[0] = CVec2D(halfWidth, halfHeight);
	//carCorners[1] = CVec2D(halfWidth, -halfHeight);
	//carCorners[2] = CVec2D(-halfWidth, -halfHeight);
	//carCorners[3] = CVec2D(-halfWidth, halfHeight);
	//draw.SetColor(0, 255, 255);
	//for (auto& corner : carCorners) {
	//	corner.Rotate(simCar.Angle);
	//	corner += simCar.Position;
	//	draw.FillCircle(corner, 5);
	//}

	////////////////////////////////////// Собственно делаем ход
	resultMove->setEnginePower(1.0);
	if (bestFirstLength > 0) {
		if (bestFirstAction == INT_MIN) {
			resultMove->setBrake(true);
		} else {
			resultMove->setWheelTurn(bestFirstAction);
		}
	} else if (bestSecondLength > 0) {
		if (bestSecondAction == INT_MIN) {
			resultMove->setBrake(true);
		} else {
			resultMove->setWheelTurn(bestSecondAction);
		}
	} else if (bestThirdLength > 0) {
		if (bestThirdAction == INT_MIN) {
			resultMove->setBrake(true);
		} else {
			resultMove->setWheelTurn(bestThirdAction);
		}
	}

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
	double turnTicks = (bestFirstAction == 1 || bestFirstAction == -1) ? bestFirstLength : 0;
	turnTicks += (bestSecondAction == 1 || bestSecondAction == -1) ? bestSecondLength : 0;
	turnTicks += (bestThirdAction == 1 || bestThirdAction == -1) ? bestThirdLength : 0;
	if (self->getNitroChargeCount() > 0 && self->getRemainingNitroCooldownTicks() == 0 && self->getRemainingNitroTicks() == 0
		&& turnTicks < 20 && bestTick > 120)
	{
		resultMove->setUseNitro(true);
	}

	///////////////////////////// статы
	//bestTickStats[bestTick] += 1;
	//bestFirstTurnStats[bestFirstTurn] += 1;
	//bestFirstLengthStats[bestFirstLength] += 1;
	//bestSecondBrakeStats[bestSecondBrake] += 1;
	//bestSecondLengthStats[bestSecondLength] += 1;
	//bestThirdTurnStats[bestThirdTurn] += 1;
	//bestThirdLengthStats[bestThirdLength] += 1;
	////logStats(bestTickStats, "bestTickStats", log.Stream());
	//logStats(bestFirstTurnStats, "bestFirstTurnStats", log.Stream());
	//logStats(bestFirstLengthStats, "bestFirstLengthStats", log.Stream());
	//logStats(bestSecondBrakeStats, "bestSecondBrakeStats", log.Stream());
	//logStats(bestSecondLengthStats, "bestSecondLengthStats", log.Stream());
	//logStats(bestThirdTurnStats, "bestThirdTurnStats", log.Stream());
	//logStats(bestThirdLengthStats, "bestThirdLengthStats", log.Stream());

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
