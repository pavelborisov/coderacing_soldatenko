#include "BestMoveFinder.h"

#include <assert.h>
#include <algorithm>
#include "DrawPlugin.h"
#include "Log.h"

using namespace std;

// TODO: Добавить четвёртый "ход", который только едет прямо.
const vector<pair<vector<CMyMove>, vector<int>>> CBestMoveFinder::allMovesWithLengths = {
	// Первое множество действий
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 },{ 1, 1 },{ -1, 1 } },
		{ 0, 5, 10, 20, 30 }
	},
	// Второе множество действий
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 },{ 1, 1 },{ -1, 1 } },
		{ 0, 20 }
	},
	// Третье множество действий
	{
		{ { 1, 0 },{ -1, 0 } },
		{ 0, 30 }
	},
	// Четвёртое действие - просто ехать по прямой (пока не упрёмся в maxTick)
	{
		{ { 0, 0 } },
		{ 1000 }
	}
};

CBestMoveFinder::CBestMoveFinder(
		const CMyCar& car,
		const model::World& world,
		const model::Game& game,
		const std::vector<CMyTile>& tileRoute,
		const CSimulator& simulator,
		const CBestMoveFinder::CResult& previousResult ) :
	car(car),
	world(world),
	game(game),
	tileRoute(tileRoute),
	simulator(simulator)
{
	bonuses = world.getBonuses();
	for (const auto& b : bonuses) {
		bonusPositions.push_back(CVec2D(b.getX(), b.getY()));
	}

	correctedPreviousMoveList = previousResult.MoveList;

	for (int i = correctedPreviousMoveList.size() - 1; i >= 0; i--) {
		correctedPreviousMoveList[i].Start--;
		if (correctedPreviousMoveList[i].End < maxTick) {
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
	CDrawPlugin::Instance().SetColor(0, 0, 255);
	const int simulationEnd = bestMoveList.back().End;
	for (int tick = 0; tick < simulationEnd; tick++) {
		model::Move simMove;
		for (const auto& m : bestMoveList) {
			if (tick >= m.Start && tick < m.End) {
				simMove = m.Move.Convert();
			}
		}
		simCar = simulator.Predict(simCar, world, simMove);
		CDrawPlugin::Instance().FillCircle(simCar.Position, 5);
	}
	const double halfHeight = game.getCarHeight() / 2;
	const double halfWidth = game.getCarWidth() / 2;
	CDrawPlugin::Instance().SetColor(0, 255, 255);
	for (const auto& corner : simCar.RotatedRect.Corners) {
		CDrawPlugin::Instance().FillCircle(corner, 5);
	}

	return result;
}

void CBestMoveFinder::processPreviousMoveList()
{
	if (correctedPreviousMoveList.size() == 0) {
		return;
	}

	CState current(car, 0, 1, 0, bonuses.size());
	CDrawPlugin::Instance().SetColor(0, 0, 255);
	const int simulationEnd = correctedPreviousMoveList.back().End;
	for (int tick = 0; tick < simulationEnd; tick++) {
		CMyMove move;
		size_t mi = 0;
		for (; mi < correctedPreviousMoveList.size(); mi++) {
			const CMoveWithDuration& m = correctedPreviousMoveList[mi];
			if (tick >= m.Start && tick < m.End) {
				move = m.Move;
				break;
			}
		}
		current.Car = simulator.Predict(current.Car, world, move.Convert());
		simulationTicks++;
		// Подсчёт очков за пройденную клетку маршрута. TODO: переделать
		if (CMyTile(current.Car.Position) == tileRoute[current.NextRouteIndex]) {
			current.RouteScore += 800;
			current.NextRouteIndex = (current.NextRouteIndex + 1) % tileRoute.size();
		}
		// Штраф за торможение в самом начале.
		if (current.Tick == 0 && move.Brake == 1) {
			current.RouteScore -= 50;
		}
		// Подбор бонусов
		processBonus(current);
		// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
		if (current.Car.CollisionDetected) {
			correctedPreviousMoveList[mi].End = tick;
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
		stateCache.push_back({ car, 0, 1, 0, bonuses.size() });
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
				// Подсчёт очков за пройденную клетку маршрута. TODO: переделать
				if (CMyTile(current.Car.Position) == tileRoute[current.NextRouteIndex]) {
					current.RouteScore += 800;
					current.NextRouteIndex = (current.NextRouteIndex + 1) % tileRoute.size();
				}
				// Штраф за торможение в самом начале.
				if (current.Tick == 0 && move.Brake == 1) {
					current.RouteScore -= 50;
				}
				// Подбор бонусов
				processBonus(current);
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

void CBestMoveFinder::processBonus(CState& state)
{
	static const double distToCenterSqrCutoff = pow(170 + 30, 2);
	static const double distToCenterSqrSure = pow(70 + 20, 2);
	static const double bonusRadiusSqr = pow(20, 2);
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
					state.RouteScore += 300;
					break;
				case model::AMMO_CRATE:
					state.RouteScore += 300;
					break;
				case model::NITRO_BOOST:
					state.RouteScore += 300;
					break;
				case model::OIL_CANISTER:
					state.RouteScore += 300;
					break;
				case model::PURE_SCORE:
					state.RouteScore += 900;
					break;
				default:
					assert(false);
			}
		}
	}
}

double CBestMoveFinder::evaluate(const CState& state) const
{
	CMyTile carTile(state.Car.Position); // В каком тайле оказались.
	CMyTile targetTile(tileRoute[state.NextRouteIndex]); // Какой следующий тайл по маршруту.
	double score = state.RouteScore; // Очки за все предыдущие полностью пройденные тайлы из маршрута.
	const int dx = targetTile.X - carTile.X;
	const int dy = targetTile.Y - carTile.Y;
	if (dx == 1 && dy == 0) {
		score += state.Car.Position.X - (carTile.X) * 800;
	} else if (dx == -1 && dy == 0) {
		score += (carTile.X + 1) * 800 - state.Car.Position.X; 
	} else if (dx == 0 && dy == 1) {
		score += state.Car.Position.Y - (carTile.Y) * 800;
	} else if (dx == 0 && dy == -1) {
		score += (carTile.Y + 1) * 800 - state.Car.Position.Y; 
	} else {
		// Где-то далеко мы находимся.
		//score -= (state.Car.Position - carTile.ToVec()).Length();
		score -= 800;
		//score -= 0; // Пока не штрафуем.
	}
	return score;
}
