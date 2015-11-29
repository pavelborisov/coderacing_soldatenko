#include "BestMoveFinder.h"

#include <assert.h>
#include <algorithm>
#include "DrawPlugin.h"
#include "Log.h"
#include "WaypointsDistanceMap.h"

using namespace std;

// TODO: Добавить четвёртый "ход", который только едет прямо.
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
	const CSimulator& simulator,
	const CBestMoveFinder::CResult& previousResult) :
	car(car),
	nextWaypointIndex(nextWaypointIndex),
	self(self),
	world(world),
	game(game),
	waypointTiles(waypointTiles),
	simulator(simulator)
{
	bonuses = world.getBonuses();
	for (const auto& b : bonuses) {
		bonusPositions.push_back(CVec2D(b.getX(), b.getY()));
	}

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
	simulationTicks = 0;
	bestScore = INT_MIN;
	bestMoveList.clear();

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
		simCar = simulator.Predict(simCar, world, simMove);
		CDrawPlugin::Instance().FillCircle(simCar.Position.X, simCar.Position.Y, 5, 0x0000FF);
	}
	const double halfHeight = game.getCarHeight() / 2;
	const double halfWidth = game.getCarWidth() / 2;
	for (const auto& corner : simCar.RotatedRect.Corners) {
		CDrawPlugin::Instance().FillCircle(corner.X, corner.Y, 5, 0x00FFFF);
	}

	return result;
}

void CBestMoveFinder::processPreviousMoveList()
{
	if (correctedPreviousMoveList.size() == 0) {
		return;
	}

	CState current(car, 0, nextWaypointIndex, 0, bonuses.size());
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
		current.Car = simulator.Predict(current.Car, world, move.Convert());
		simulationTicks++;

		processRouteScore(current, current.Tick == 0 && move.Brake == 1);

		// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
		if (current.Car.CollisionDetected) {
			correctedPreviousMoveList[mi].End = current.Tick;
			correctedPreviousMoveList.erase(correctedPreviousMoveList.begin() + mi + 1, correctedPreviousMoveList.end());
			break;
		}
	}
	double score = evaluate(current);
	if (score > bestScore) {
		bestScore = score;
		bestMoveList = correctedPreviousMoveList;
	}
}

void CBestMoveFinder::processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList)
{
	const bool lastMove = moveIndex == allMovesWithLengths.size() - 1;
	const vector<CMyMove>& moveArray = allMovesWithLengths[moveIndex].first;
	const vector<int>& lengthsArray = allMovesWithLengths[moveIndex].second;

	if (moveIndex == 0) {
		stateCache.clear();
		stateCache.push_back({ car, 0, nextWaypointIndex, 0, bonuses.size() });
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
				current.Car = simulator.Predict(current.Car, world, move.Convert());
				simulationTicks++;

				processRouteScore(current, current.Tick == 0 && move.Brake == 1);

				// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
				if (current.Car.CollisionDetected) {
					break;
				}
				if (lastMove) {
					double score = evaluate(current);
					if (score > bestScore) {
						bestScore = score;
						bestMoveList = prevMoveList;
						bestMoveList.push_back({ move, start, current.Tick });
					}
				}
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
	// Подбор бонусов
	processBonus(state);
}

void CBestMoveFinder::processBonus(CState& state)
{
	static const double distToCenterSqrCutoff = pow(170 + 30, 2);
	static const double distToCenterSqrSure = pow(70 + 20, 2);
	//static const double bonusRadiusSqr = pow(20, 2);
	static const double bonusRadiusSqr = pow(18, 2); // -2 для точности
	for (size_t i = 0; i < bonuses.size(); i++) {
		if (state.PickedBonuses[i]) continue;
		const double distToCenterSqr = (bonusPositions[i] - state.Car.Position).LengthSquared();
		if (distToCenterSqr > distToCenterSqrCutoff) continue;
		bool pickedUp = false;
		if (distToCenterSqr < distToCenterSqrSure) {
			pickedUp = true;
		}
		if (!pickedUp) {
			for (const auto& c : state.Car.RotatedRect.Corners) {
				const double distSqr = (bonusPositions[i] - c).LengthSquared();
				if (distSqr < bonusRadiusSqr) {
					pickedUp = true;
					break;
				}
			}
		}
		if (pickedUp) {
			state.PickedBonuses[i] = true;
			switch (bonuses[i].getType()) {
				case model::REPAIR_KIT:
					// TODO: Durability
					state.RouteScore += self.getDurability() < 0.3 ? 700 : 100;
					break;
				case model::AMMO_CRATE:
					state.RouteScore += 200;
					break;
				case model::NITRO_BOOST:
					state.RouteScore += 300;
					break;
				case model::OIL_CANISTER:
					state.RouteScore += 100;
					break;
				case model::PURE_SCORE:
					state.RouteScore += 700;
					break;
				default:
					assert(false);
			}
		}
	}
}

double CBestMoveFinder::evaluate(const CState& state) const
{
	const CVec2D carDirectionUnitVector(cos(state.Car.Angle), sin(state.Car.Angle));
	const double angle = (state.Car.Speed + carDirectionUnitVector).GetAngle();
	const double dist = CWaypointDistanceMap::Instance().Query(
		state.Car.Position.X, state.Car.Position.Y, angle, state.NextWaypointIndex);
	if (dist >= 0) {
		return state.RouteScore - dist;
	} else {
		//assert(false);
		return -1000000;
	}
}
