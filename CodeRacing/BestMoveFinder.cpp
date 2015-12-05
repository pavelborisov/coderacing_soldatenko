#include "BestMoveFinder.h"

#include <assert.h>
#include <algorithm>
#include "DrawPlugin.h"
#include "GlobalPredictions.h"
#include "Log.h"
#include "WaypointsDistanceMap.h"

using namespace std;

static const int maxCollisionsDetected = 1;
//static const double veryBadScoreDif = -10000;
static const double veryBadScoreDif = -1000000;
static const int carSimulationSubticksCount = 2;

const vector<pair<vector<CMyMove>, vector<int>>> CBestMoveFinder::allMovesWithLengths = {
	// Первое множество действий
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 },{ 1, 1 },{ -1, 1 } },
		{ 0, 5, 10, 20, 40 }
	},
	// Второе множество действий
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 },{ 1, 1 },{ -1, 1 } },
		{ 0, 40 }
	},
	// Третье множество действий
	{
		{ { 1, 0 },{ -1, 0 } },
		{ 0, 40 }
	},
	// Четвёртое действие - просто ехать по прямой (пока не упрёмся в maxTick)
	{
		{ { 0, 0 } },
		{ 1000 }
	}
};

CBestMoveFinder::CBestMoveFinder(
	const CMyCar& car,
	int nextWaypointIndex,
	const model::Car& self,
	const model::World& world,
	const model::Game& game,
	const std::vector<CMyTile>& waypointTiles,
	CSimulator& simulator,
	const CBestMoveFinder::CResult& previousResult) :
	car(car),
	nextWaypointIndex(nextWaypointIndex),
	self(self),
	world(world),
	game(game),
	waypointTiles(waypointTiles),
	simulator(simulator)
{
	correctedPreviousMoveList = previousResult.MoveList;

	for (int i = correctedPreviousMoveList.size() - 1; i >= 0; i--) {
		correctedPreviousMoveList[i].Start--;
		if (correctedPreviousMoveList[i].End < maxTick - 1) {
			correctedPreviousMoveList[i].End--;
		}
		if (correctedPreviousMoveList[i].End < 0) {
			correctedPreviousMoveList.erase(correctedPreviousMoveList.begin() + i);
		}
	}
}

CBestMoveFinder::CResult CBestMoveFinder::Process()
{
	simulator.SetPrecision(carSimulationSubticksCount);
	simulationTicks = 0;
	bestScore = INT_MIN;
	bestMoveList.clear();

	CState start(car, 0, nextWaypointIndex, 0, CGlobalPredictions::Bonuses.size());
	startScore = evaluate(start);

	CResult result;
	processPreviousMoveList();
	processMoveIndex(0, vector<CMoveWithDuration>());
	if (bestMoveList.size() == 0) {
		result.Success = false;
		return result;
	}
	result.Success = true;
	result.MoveList = bestMoveList;
	for (const auto& m : bestMoveList) {
		if (m.End > 0) {
			result.CurrentMove = m.Move;
			break;
		}
	}
	result.Score = bestScore;

	/////////////////////////////// log
	static int totalSimulationTicks = 0;
	totalSimulationTicks += simulationTicks;
	CLog::Instance().Log(simulationTicks, "SimulationTicks");
	CLog::Instance().Log(totalSimulationTicks, "TotalSimulationTicks");
	CLog::Instance().Stream() << "BestMoveList (Score = " << bestScore << "):" << endl;
	for (const auto& m : bestMoveList) {
		CLog::Instance().Stream() << "  Turn:" << m.Move.Turn << " Brake:" << m.Move.Brake << " Engine:" << m.Move.Engine << " Start:" << m.Start << " End:" << m.End << endl;
	}

	////////////////////////////////////////// draw best
	CMyCar simCar = car;
	const int simulationEnd = bestMoveList.back().End;
	for (int tick = 0; tick < simulationEnd; tick++) {
		model::Move simMove;
		for (const auto& m : bestMoveList) {
			if (tick >= m.Start && tick < m.End) {
				simMove = m.Move.Convert();
			}
		}
		simCar = simulator.Predict(simCar, simMove, tick);
		CDrawPlugin::Instance().FillCircle(simCar.Position.X, simCar.Position.Y, 5, 0x0000FF);
	}
	const double halfHeight = game.getCarHeight() / 2;
	const double halfWidth = game.getCarWidth() / 2;
	for (const auto& corner : simCar.RotatedRect.Corners) {
		CDrawPlugin::Instance().FillCircle(corner.X, corner.Y, 5, 0x00FFFF);
	}

	return result;
}

static bool chance(int percentage)
{
	return rand() % 100 < percentage;
}

static int uniform(int start, int end)
{
	return start + rand() % (end - start);
}

void CBestMoveFinder::processPreviousMoveList()
{
	if (correctedPreviousMoveList.size() == 0) {
		return;
	}

	CState current(car, 0, nextWaypointIndex, 0, CGlobalPredictions::Bonuses.size());
	const int simulationEnd = correctedPreviousMoveList.back().End;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove move;
		size_t mi = 0;
		for (; mi < correctedPreviousMoveList.size(); mi++) {
			const CMoveWithDuration& m = correctedPreviousMoveList[mi];
			if (current.Tick >= m.Start && current.Tick < m.End) {
				move = m.Move;
				break;
			}
		}
		current.Car = simulator.Predict(current.Car, move.Convert(), current.Tick);
		simulationTicks++;

		processRouteScore(current, current.Tick == 0 && move.Brake == 1);

		// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
		if (current.Car.CollisionsDetected > maxCollisionsDetected) {
			correctedPreviousMoveList[mi].End = current.Tick;
			correctedPreviousMoveList.erase(correctedPreviousMoveList.begin() + mi + 1, correctedPreviousMoveList.end());
			break;
		}

		double score = evaluate(current);
		if (score > bestScore) {
			bestScore = score;
			bestMoveList = correctedPreviousMoveList;
		}
	}
}

void CBestMoveFinder::processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList)
{
	//bool lastMove = moveIndex == allMovesWithLengths.size() - 1;
	//const vector<CMyMove>& moveArray = allMovesWithLengths[moveIndex].first;
	//const vector<int>& lengthsArray = allMovesWithLengths[moveIndex].second;
	vector<CMyMove> moveArray;
	vector<int> lengthsArray;
	bool lastMove = moveIndex == 3;

	if (moveIndex == 0) {
		moveArray.push_back({ 0, 0 });
		if (chance(50)) {
			moveArray.push_back({ 1, 0 });
			moveArray.push_back({ 1, 1 });
		} else {
			moveArray.push_back({ -1, 0 });
			moveArray.push_back({ -1, 1 });
		}
		lengthsArray.push_back(0);
		lengthsArray.push_back(uniform(2, 7));
		lengthsArray.push_back(uniform(7, 15));
		lengthsArray.push_back(uniform(15, 30));
		lengthsArray.push_back(uniform(30, 50));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 1) {
		moveArray.push_back({ 0, 0 });
		if (chance(50)) {
			moveArray.push_back({ 1, 0 });
			moveArray.push_back({ 1, 1 });
		} else {
			moveArray.push_back({ -1, 0 });
			moveArray.push_back({ -1, 1 });
		}
		lengthsArray.push_back(0);
		lengthsArray.push_back(uniform(30, 50));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 2) {
		moveArray.push_back({ 0, 0 });
		if (chance(50)) {
			moveArray.push_back({ 1, 0 });
		} else {
			moveArray.push_back({ -1, 0 });
		}
		lengthsArray.push_back(0);
		lengthsArray.push_back(uniform(30, 50));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 3) {
		moveArray.push_back({ 0, 0 });
		lengthsArray.push_back(1000);
		sort(lengthsArray.begin(), lengthsArray.end());
	}

	if (moveIndex == 0) {
		stateCache.clear();
		stateCache.push_back({ car, 0, nextWaypointIndex, 0, CGlobalPredictions::Bonuses.size() });
	}

	for (const CMyMove& move : moveArray) {
		CState current = stateCache.back();
		const int start = current.Tick;
		for (int length : lengthsArray) {
			if (length == 0 && move != moveArray[0]) {
				// Если у действия длина == 0, то нам не нужно считать несколько действий. Хватит только первого из массива.
				continue;
			}
			const int end = min(maxTick, start + length);
			assert(current.Tick <= end); // Проверка правильного порядка в массиве lenghtsArray
			// Продолжаем симулировать с того места, откуда закончили в предыдущий раз.
			for (; current.Tick < end; current.Tick++) {
				current.Car = simulator.Predict(current.Car, move.Convert(), current.Tick);
				simulationTicks++;

				processRouteScore(current, current.Tick == 0 && move.Brake == 1);

				// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
				if (current.Car.CollisionsDetected > maxCollisionsDetected) {
					break;
				}
				//if (lastMove) {
					double score = evaluate(current);
					if (score > bestScore) {
						bestScore = score;
						bestMoveList = prevMoveList;
						bestMoveList.push_back({ move, start, current.Tick });
					} else if (score - startScore < veryBadScoreDif) {
						// Отсечение по очкам
						break;
					}
				//}
			}

			if (!lastMove) {
				// Запускаем обработку следующего действия.
				vector<CMoveWithDuration> moveList = prevMoveList;
				moveList.push_back({ move, start, end });
				stateCache.push_back(current);
				processMoveIndex(moveIndex + 1, moveList);
				stateCache.pop_back();
			}

			if (end == maxTick) {
				// Нет смысла больше увеличивать длину действия, т.к. мы уже итак досчитали до ограничения по максимальному кол-ву тиков.
				break;
			}
		}
	}
}

void CBestMoveFinder::processRouteScore(CState& state, bool firstTickBrake)
{
	// Подсчёт очков за пройденную клетку маршрута.
	const CMyTile& nextWaypointTile = waypointTiles[state.NextWaypointIndex];
	if (CMyTile(state.Car.Position) == nextWaypointTile) {
		const int afterNextWaypointIndex = (state.NextWaypointIndex + 1) % waypointTiles.size();
		const CVec2D nextWaypointVec = waypointTiles[state.NextWaypointIndex].ToVec();
		const double dist = CWaypointDistanceMap::Instance().QueryBestDirection(nextWaypointVec.X, nextWaypointVec.Y, afterNextWaypointIndex);
		assert(dist >= 0);
		state.RouteScore += dist;
		state.RouteScore += 400;
		state.NextWaypointIndex = afterNextWaypointIndex;
	}
	// Штраф за торможение в самом начале.
	if (firstTickBrake) {
		state.RouteScore -= 200;
	}
	// Штраф за въезд в лужу. Растёт от скорости.
	if (state.Car.OiledTicks == 59 && state.Car.Speed.Length() > 10) {
		state.RouteScore -= (state.Car.Speed.Length() - 10) * 100;
	}
	// Подбор бонусов
	processBonus(state);
}

void CBestMoveFinder::processBonus(CState& state)
{
	static const double distToCenterSqrCutoff = pow(170 + 30, 2);
	static const double distToCenterSqrSure = pow(70 + 20, 2);
	//static const double bonusRadiusSqr = pow(20, 2);
	static const double bonusRadiusSqr = pow(18, 2); // -2 для точности
	for (size_t i = 0; i < CGlobalPredictions::Bonuses.size(); i++) {
		if (state.PickedBonuses[i]) continue;
		const CMyBonus& bonus = CGlobalPredictions::Bonuses[i];
		//if (state.Tick >= bonus.LastTick) continue;
		const double distToCenterSqr = (bonus.Position - state.Car.Position).LengthSquared();
		if (distToCenterSqr > distToCenterSqrCutoff) continue;
		bool pickedUp = false;
		if (distToCenterSqr < distToCenterSqrSure) {
			pickedUp = true;
		}
		if (!pickedUp) {
			for (const auto& c : state.Car.RotatedRect.Corners) {
				const double distSqr = (bonus.Position - c).LengthSquared();
				if (distSqr < bonusRadiusSqr) {
					pickedUp = true;
					break;
				}
			}
		}
		if (pickedUp) {
			state.PickedBonuses[i] = true;
			switch (bonus.Type) {
				case model::REPAIR_KIT:
					state.RouteScore += self.getDurability() < 0.3 ? 1500 : 200;
					break;
				case model::AMMO_CRATE:
					state.RouteScore += 400;
					break;
				case model::NITRO_BOOST:
					state.RouteScore += 1000;
					break;
				case model::OIL_CANISTER:
					state.RouteScore += 200;
					break;
				case model::PURE_SCORE:
					state.RouteScore += 1500;
					break;
				default:
					assert(false);
			}
		}
	}
}

double CBestMoveFinder::evaluate(const CState& state) const
{
	//const CVec2D carDirectionUnitVector(cos(state.Car.Angle), sin(state.Car.Angle));
	//const double angle = (state.Car.Speed + carDirectionUnitVector).GetAngle();
	const double angle = state.Car.Angle;
	const double dist = CWaypointDistanceMap::Instance().Query(
		state.Car.Position.X, state.Car.Position.Y, angle, state.NextWaypointIndex);
	double score = 0;
	if (dist >= 0) {
		score += state.RouteScore - dist;
	} else {
		//assert(false);
		score += -1000000;
	}
	// Штраф за потерю хп.
	if (state.Car.Durability < 1e-7) {
		score += -1000000;
	} else {
		// == -20000 при durability == 1 и == -200000 при durability == 0.01
		score += -20000 * sqrt(1 / state.Car.Durability);
	}
	//score += 20000 * state.Car.Durability;
	//if (state.Car.Durability > 1e-7) score += 100000;

	return score;
}
