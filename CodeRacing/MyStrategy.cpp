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

	// TODO: ������� ����� �������� � ����������� �� ����������� �������� ������ � ��������� �����.
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

// TODO: ������� ���� ������� � ��������� ����
struct CMyMove {
	int Turn = 0;   // 0, 1, -1
	int Brake = 0;  // 0, 1
	int Engine = 1; // 1, -1

	CMyMove() {}
	CMyMove(int Turn, int Brake) : Turn(Turn), Brake(Brake) {}
	CMyMove(int Turn, int Brake, int Engine) : Turn(Turn), Brake(Brake), Engine(Engine) {}

	model::Move Convert() const;
};

model::Move CMyMove::Convert() const
{
	model::Move result;
	result.setWheelTurn(Turn);
	result.setBrake(Brake == 1);
	result.setEnginePower(Engine);
	return result;
}

void MyStrategy::makeMove()
{
	// TODO: ��� � ������������� ���������� � ��������.
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(1.0);
		return;
	}

	static int totalSimulationTicks = 0;
	int simulationTicks = 0;

	double bestScore = INT_MIN;
	CMyCar bestSimCar;
	int bestTick = INT_MAX;
	CMyMove bestFirstMove;
	int bestFirstTick= 0;
	CMyMove bestSecondMove;
	int bestSecondTick = 0;
	CMyMove bestThirdMove;
	int bestThirdTick = 0;

	CMyMove firstMoveArray[] = { {0, 0}, {-1, 0}, {1, 0} };
	int firstLengthArray[] = { 0, 5, 10, 20, 30 };
	CMyMove secondMoveArray[] = { {0, 0}, {0, 1} };
	int secondLengthArray[] = { 0, 30 };
	CMyMove thirdMoveArray[] = { {0, 0}, {-1, 0}, {1, 0} };
	int thirdLengthArray[] = { 0, 10, 25, 50, 100 };

	// TODO: ������� �� ���������.
	// TODO: ������ �� ������ ��������� ��� ������ �� ���������?
	// TODO: ���� �������� �� ����� ���� ���� �� ������ �������, � ������ �������� �� ����� �������� � ����, �� ����
	//       ��� �������� ��������� "����������" �������� �� ������ ��������.
	// TODO: ��� ������ - ������������ nextWP, ������� ����� ���������� �� ������� wp �� ������������ ��������� (� ��������� �� 1/4 ������) - ������� ����� �� ��.
	// TODO: ���� ����� �������� == 0, �� ��������� ������ ������ �������� �� �������.
	for (CMyMove firstMove : firstMoveArray) {
		// ��������� ��������� �� ������ ���� ������ �������� ��������� ������� �� �����.
		const int firstStart = 0;
		// ��� ������� ��������.
		int firstTick = 0; // �� ����� ���� ���� ��������� ���������� ��������� ������� ��������.
		CMyCar firstCar = car; // ���������, � ������� �� ������ ����� ���������� ��������� ������� ��������.
		double firstRouteScore = 0;
		int firstRouteIndex = 1;
		// ������� ����� ��������. ������ ���� � ������� ���������� �����, ����� ������������ ���.
		for (int firstLength : firstLengthArray) {
			const int firstEnd = firstStart + firstLength;
			// ���������� �� ������������� ������� � firstLengthArray.
			assert(firstTick <= firstEnd);
			// ���������� ������ ������ �������� �� ��� �����. ��� ���� ���� ����� ��� "������", ��
			// �� ��������� ��������� � ����������� ����������.
			for (; firstTick < firstEnd; firstTick++) {
				firstCar = simulator.Predict(firstCar, *world, firstMove.Convert());
				simulationTicks++;
				// ������� ����� �� ���������� ������ ��������. TODO: ����������
				if (CMyTile(firstCar.Position) == tileRoute[firstRouteIndex]) {
					firstRouteScore += 800;
					firstRouteIndex++;
				}
				if (firstCar.CollisionDetected) break; // ������� �� ���������. TODO: �� ���������������, ���� �������� ���� "������"
			}

			// ������ ��������.
			for (CMyMove secondMove : secondMoveArray) {
				const int secondStart = firstTick;
				// ��� ������� ��������.
				int secondTick = firstTick;
				CMyCar secondCar = firstCar;
				double secondRouteScore = firstRouteScore;
				int secondRouteIndex = firstRouteIndex;
				for (int secondLength : secondLengthArray) {
					const int secondEnd = secondStart + secondLength;
					assert(secondTick <= secondEnd);
					for (; secondTick < secondEnd; secondTick++) {
						secondCar = simulator.Predict(secondCar, *world, secondMove.Convert());
						simulationTicks++;
						// ������� ����� �� ���������� ������ ��������.
						if (CMyTile(secondCar.Position) == tileRoute[secondRouteIndex]) {
							secondRouteScore += 800;
							secondRouteIndex++;
						}
						if (secondCar.CollisionDetected) break; // ������� �� ���������
					}

					// ������ ��������.
					for (CMyMove thirdMove : thirdMoveArray) {
						const int thirdStart = secondTick;
						// ��� �������� ��������.
						int thirdTick = secondTick;
						CMyCar thirdCar = secondCar;
						double thirdRouteScore = secondRouteScore;
						int thirdRouteIndex = secondRouteIndex;
						for (int thirdLength : thirdLengthArray) {
							const int thirdEnd = thirdStart + thirdLength;
							assert(thirdTick <= thirdEnd);
							for (; thirdTick < thirdEnd; thirdTick++) {
								thirdCar = simulator.Predict(thirdCar, *world, thirdMove.Convert());
								simulationTicks++;
								// ������� ����� �� ���������� ������ ��������.
								if (CMyTile(thirdCar.Position) == tileRoute[thirdRouteIndex]) {
									thirdRouteScore += 800;
									thirdRouteIndex++;
								}
								if (thirdCar.CollisionDetected) break; // ������� �� ���������
							}

							// ������ �������� �������.
							CMyCar simCar = thirdCar; // �������� ���������.
							CMyTile simCarTile(thirdCar.Position); // � ����� ����� ���������.
							CMyTile targetTile(tileRoute[thirdRouteIndex]); // ����� ��������� ���� �� ��������.
							double score = thirdRouteScore; // ���� �� ��� ���������� ��������� ���������� �����.
							// ���� �� �������� ���������� ����, ���� �� ��������� � �������� ����� �� ��������.
							if (simCarTile.X == targetTile.X + 1) {
								score += (simCarTile.X + 1) * 800 - simCar.Position.X;
							} else if (simCarTile.X == targetTile.X - 1) {
								score += simCar.Position.X - (simCarTile.X) * 800;
							} else if (simCarTile.Y == targetTile.Y + 1) {
								score += (simCarTile.Y + 1) * 800 - simCar.Position.Y;
							} else if (simCarTile.Y == targetTile.Y - 1) {
								score += simCar.Position.Y - (simCarTile.Y) * 800;
							} else {
								// ���-�� ������ �� ���������.
								score -= 0; // ���� �� ��������.
							}
							// TODO: �������������� ����� �� ����������?

							// �������� ���������� ��������.
							if (score > bestScore) {
								bestScore = score;
								bestTick = thirdTick;
								bestSimCar = simCar;
								bestFirstMove = firstMove;
								bestFirstTick = firstTick;
								bestSecondMove = secondMove;
								bestSecondTick = secondTick;
								bestThirdMove = thirdMove;
								bestThirdTick = thirdTick;
							}
						}
					}
				}
			}
		}
	}
	totalSimulationTicks += simulationTicks;
	log.Log(simulationTicks, "SimulationTicks");
	log.Log(totalSimulationTicks, "TotalSimulationTicks");

	////////////////////////////////////////// draw best
	CMyCar simCar = car;
	draw.SetColor(0, 0, 255);
	for (int tick = 0; tick < bestTick; tick++) {
		model::Move simMove;

		if (tick < bestFirstTick) {
			simMove = bestFirstMove.Convert();
		} else if (tick < bestSecondTick) {
			simMove = bestSecondMove.Convert();
		} else if (tick < bestThirdTick) {
			simMove = bestThirdMove.Convert();
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
	for (auto& corner : carCorners) {
		corner.Rotate(simCar.Angle);
		corner += simCar.Position;
		draw.FillCircle(corner, 5);
	}

	////////////////////////////////////// ���������� ������ ���
	if (bestFirstTick > 0) {
		*resultMove = bestFirstMove.Convert();
	} else if (bestSecondTick > 0) {
		*resultMove = bestSecondMove.Convert();
	} else if (bestThirdTick > 0) {
		*resultMove = bestThirdMove.Convert();
	}

	// ����� ������ ���
	// TODO: ���������, ���� �� ������������ ������ � ��������� ��� � ������ ����� ������ �������.
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
			// ����� ���������
			if (self->getProjectileCount() > 0
				&& (abs(angle) < PI / 90 && dist < 1500)
					||(abs(angle) < PI/30 && dist < 700)
					||(abs(angle) < PI/9 && dist < 300 ))
			{
					resultMove->setThrowProjectile(true);
			}
			// ����� ����
			if (world->getTick() > 350 && self->getOilCanisterCount() > 0 && abs(self->getWheelTurn()) > 0.5) {
				resultMove->setSpillOil(true);
			}
		}
	}
	// TODO: �������� �� ������ �������.
	// ����� �����.
	// ������� ����� ������������.
	double turnTicks = (bestFirstMove.Turn != 0) ? bestFirstTick : 0;
	turnTicks += (bestSecondMove.Turn != 0) ? bestSecondTick - bestFirstTick : 0;
	turnTicks += (bestThirdMove.Turn != 0) ? bestThirdTick - bestSecondTick : 0;
	if (self->getNitroChargeCount() > 0 && self->getRemainingNitroCooldownTicks() == 0 && self->getRemainingNitroTicks() == 0
		&& turnTicks < 20 && bestTick > 120)
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
