#include "BestMoveFinder.h"

#include <assert.h>
#include <algorithm>
#include "DrawPlugin.h"
#include "Log.h"
#include "WaypointsDistanceMap.h"
#include "WorldSimulator.h"

using namespace std;

static const int maxCollisionsDetected = 1;
//static const double veryBadScoreDif = -10000;
static const double veryBadScoreDif = -1000000;
//static const int carSimulationSubticksCount = 2;
static const int preciseSimulationMaxTick = 40;

static void setSimulatorMode(int tick)
{
	CWorldSimulator::Instance().SetPrecision(tick > preciseSimulationMaxTick ? 1 : 2);
	if (tick > preciseSimulationMaxTick) {
		CWorldSimulator::Instance().SetOptions(true, true, true);
	} else {
		CWorldSimulator::Instance().SetOptions(false, false, false);
		//CWorldSimulator::Instance().SetOptions(true, true, true);
	}

}

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
	const CMyWorld& startWorld,
	const std::vector<CMyTile>& waypointTiles,
	const CBestMoveFinder::CResult& previousResult) :
	startWorld(startWorld),
	waypointTiles(waypointTiles)
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
	simulationTicks = 0;
	bestScore = INT_MIN;
	bestMoveList.clear();

	CState start(startWorld, 0, 0);
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
	postProcess(result);

#ifdef LOGGING
	/////////////////////////////// log
	static int totalSimulationTicks = 0;
	totalSimulationTicks += simulationTicks;
	CLog::Instance().Log(simulationTicks, "SimulationTicks");
	CLog::Instance().Log(totalSimulationTicks, "TotalSimulationTicks");
	CLog::Instance().Stream() << "BestMoveList (Score = " << bestScore << "):" << endl;
	for (const auto& m : bestMoveList) {
		CLog::Instance().Stream() << "  Turn:" << m.Move.Turn << " Brake:" << m.Move.Brake << " Engine:" << m.Move.Engine << " Start:" << m.Start << " End:" << m.End << endl;
	}

	//////////////////////////////////////////// draw best
	CMyWorld simWorld = startWorld;
	const int simulationEnd = bestMoveList.back().End;
	for (int tick = 0; tick < simulationEnd; tick++) {
		CMyMove moves[4];
		for (const auto& m : bestMoveList) {
			if (tick >= m.Start && tick < m.End) {
				moves[0] = m.Move;
			}
		}
		setSimulatorMode(tick);
		simWorld = CWorldSimulator::Instance().Simulate(simWorld, moves);
		//int color = min(200, 100 + tick);
		//simWorld.Draw(0xFF00FF + 0x000100 * color);
		CDrawPlugin::Instance().FillCircle(simWorld.Cars[0].Position.X, simWorld.Cars[0].Position.Y, 5, 0x0000FF);
	}
#endif

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

	CState current(startWorld, 0, 0);
	const int simulationEnd = correctedPreviousMoveList.back().End;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove moves[4];
		size_t mi = 0;
		for (; mi < correctedPreviousMoveList.size(); mi++) {
			const CMoveWithDuration& m = correctedPreviousMoveList[mi];
			if (current.Tick >= m.Start && current.Tick < m.End) {
				moves[0] = m.Move;
				break;
			}
		}
		setSimulatorMode(current.Tick);
		current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;

		processRouteScore(current, current.Tick == 0 && moves[0].Brake == true);

		// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
		if (current.World.Cars[0].CollisionsDetected > maxCollisionsDetected) {
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
			if (chance(50)) moveArray.push_back({ 1, 1 });
		} else {
			moveArray.push_back({ -1, 0 });
			if (chance(50)) moveArray.push_back({ -1, 1 });
		}
		lengthsArray.push_back(0);
		lengthsArray.push_back(uniform(4, 12));
		lengthsArray.push_back(uniform(12, 30));
		lengthsArray.push_back(uniform(30, 50));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 1) {
		moveArray.push_back({ 0, 0 });
		if (chance(50)) {
			moveArray.push_back({ 1, 0 });
			if (chance(50)) moveArray.push_back({ 1, 1 });
		} else {
			moveArray.push_back({ -1, 0 });
			if (chance(50)) moveArray.push_back({ -1, 1 });
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
		stateCache.push_back({ startWorld, 0, 0 });
	}

	CMyMove moves[4];
	for (const CMyMove& move : moveArray) {
		moves[0] = move;
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
				setSimulatorMode(current.Tick);
				current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
				simulationTicks++;

				processRouteScore(current, current.Tick == 0 && move.Brake == 1);

				// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
				if (current.World.Cars[0].CollisionsDetected > maxCollisionsDetected) {
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
	CMyCar& car = state.World.Cars[0];
	const CMyTile& nextWaypointTile = waypointTiles[car.NextWaypointIndex];
	if (CMyTile(car.Position) == nextWaypointTile) {
		const int afterNextWaypointIndex = (car.NextWaypointIndex + 1) % waypointTiles.size();
		const CVec2D nextWaypointVec = waypointTiles[car.NextWaypointIndex].ToVec();
		const double dist = CWaypointDistanceMap::Instance().QueryBestDirection(nextWaypointVec.X, nextWaypointVec.Y, afterNextWaypointIndex);
		assert(dist >= 0);
		state.RouteScore += dist;
		state.RouteScore += 400;
		car.NextWaypointIndex = afterNextWaypointIndex;
	}

	// Штраф за торможение в самом начале.
	if (firstTickBrake) {
		state.RouteScore -= 200;
	}

	// Штраф за въезд в лужу. Растёт от скорости.
	if (car.OiledTicks == 59 && car.Speed.Length() > 12) {
		state.RouteScore -= (car.Speed.Length() - 12) * 100;
	}

	// Подбор бонусов
	processBonus(state);
}

void CBestMoveFinder::processBonus(CState& /*state*/)
{
	//const CMyCar& startCar = startWorld.Cars[0];
	//const CMyCar& car = state.World.Cars[0];
	//const double durabilityDifPositive = max(0.0, car.Durability - startCar.Durability);
	//const int ammoDifPositive = max(0, car.ProjectilesCount - startCar.ProjectilesCount);
	//const int nitroDifPositive = max(0, car.NitroCount - startCar.NitroCount);
	//const int oilDifPositive = max(0, car.OilCount - startCar.OilCount);
	//const int moneyDifPositive = max(0, car.MoneyCount - startCar.MoneyCount);
	////int scoreDif = state.World.Players[0].Score - startWorld.Players[0].Score;

	//state.RouteScore += durabilityDifPositive * 1500;
	//state.RouteScore += ammoDifPositive * 400;
	//state.RouteScore += nitroDifPositive * 1000;
	//state.RouteScore += oilDifPositive * 200;
	////state.RouteScore += scoreDif * 15;
	//state.RouteScore += moneyDifPositive * 1500;
}

double CBestMoveFinder::evaluate(const CState& state) const
{
	const CMyCar& startCar = startWorld.Cars[0];
	const CMyCar& car = state.World.Cars[0];
	const double angle = car.Angle;
	const double dist = CWaypointDistanceMap::Instance().Query(
		car.Position.X, car.Position.Y, angle, car.NextWaypointIndex);
	double score = 0;
	if (dist >= 0) {
		score += state.RouteScore - dist;
	} else {
		//assert(false);
		score += -1000000;
	}

	// Штраф за потерю хп.
	if (car.Durability < 1e-7) {
		score += -1000000;
	} else {
		// == -20000 при durability == 1 и == -200000 при durability == 0.01
		score += -20000 * sqrt(1 / car.Durability);
	}

	const double durabilityDifPositive = max(0.0, car.Durability - startCar.Durability);
	const int ammoDifPositive = max(0, car.ProjectilesCount - startCar.ProjectilesCount);
	const int nitroDifPositive = max(0, car.NitroCount - startCar.NitroCount);
	const int oilDifPositive = max(0, car.OilCount - startCar.OilCount);
	const int moneyDifPositive = max(0, car.MoneyCount - startCar.MoneyCount);
	//int scoreDif = state.World.Players[0].Score - startWorld.Players[0].Score;

	score += durabilityDifPositive * 1500;
	score += ammoDifPositive * 400;
	score += nitroDifPositive * 1000;
	score += oilDifPositive * 200;
	score += moneyDifPositive * 1500;
	//score += scoreDif * 15;

	return score;
}

void CBestMoveFinder::postProcess(CResult& result)
{
	CWorldSimulator::Instance().SetOptions(false, false, false);
	CWorldSimulator::Instance().SetPrecision(2);
	CState current(startWorld, 0, 0);
	const int simulationEnd = 40;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove moves[4];
		size_t mi = 0;
		for (; mi < correctedPreviousMoveList.size(); mi++) {
			const CMoveWithDuration& m = correctedPreviousMoveList[mi];
			if (current.Tick >= m.Start && current.Tick < m.End) {
				moves[0] = m.Move;
				break;
			}
		}
		current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;
	}

	postProcessShooting(current, result);
	postProcessOil(current, result);
}

void CBestMoveFinder::postProcessShooting(const CState& before, CResult& result)
{
	const auto& car = startWorld.Cars[0];
	if (car.ProjectilesCount == 0 || car.ProjectileCooldown > 0) {
		return;
	}

	double myPlayerDamageScoreBefore = 0;
	double enemyPlayerDamageScoreBefore = 0;
	// TODO
	//double allyDurabilitySum = 0;
	//int allyDead = 0;
	//double enemyDurabilitySum = 0;
	//int enemyDead = 0;
	for (int i = 0; i < CMyWorld::PlayersCount; i++) {
		if (i == 0) {
			myPlayerDamageScoreBefore += before.World.Players[i].DamageScore;
		} else {
			enemyPlayerDamageScoreBefore += before.World.Players[i].DamageScore;
		}
	}

	CState current(startWorld, 0, 0);
	const int simulationEnd = before.Tick;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove moves[4];
		size_t mi = 0;
		for (; mi < correctedPreviousMoveList.size(); mi++) {
			const CMoveWithDuration& m = correctedPreviousMoveList[mi];
			if (current.Tick >= m.Start && current.Tick < m.End) {
				moves[0] = m.Move;
				break;
			}
		}
		if (current.Tick == 0) {
			moves[0].Shoot = true;
		}
		current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;
	}

	double myPlayerDamageScoreAfter = 0;
	double enemyPlayerDamageScoreAfter = 0;
	for (int i = 0; i < CMyWorld::PlayersCount; i++) {
		if (i == 0) {
			myPlayerDamageScoreAfter += current.World.Players[i].DamageScore;
		} else {
			enemyPlayerDamageScoreAfter += current.World.Players[i].DamageScore;
		}
	}

	double totalScoreDif = myPlayerDamageScoreAfter - myPlayerDamageScoreBefore;
		//+ enemyPlayerScoreBefore - enemyPlayerScoreAfter;

	CLog::Instance().Stream() << "totalScoreDif = " << totalScoreDif
		<< " = " << myPlayerDamageScoreAfter << " - " << myPlayerDamageScoreBefore
		//<< " + " << enemyPlayerScoreBefore << " - " << enemyPlayerScoreAfter
		<< endl;

	if (car.Type == 0) {
		if (car.ProjectilesCount > 1) {
			if (totalScoreDif > 40) {
				result.CurrentMove.Shoot = true;
			}
		} else {
			if (totalScoreDif > 50) {
				result.CurrentMove.Shoot = true;
			}
		}
	} else {
		if (totalScoreDif > 30) {
			result.CurrentMove.Shoot = true;
		}
	}
}

void CBestMoveFinder::postProcessOil(const CState& before, CResult& result)
{
	const auto& car = startWorld.Cars[0];
	if (car.OilCount == 0 || car.OilCooldown > 0) {
		return;
	}

	double enemyMaxOiledSpeedBefore = 0;
	double enemyDurabilitySumBefore = 0;
	for (int i = 1; i < CMyWorld::MaxCars; i++) {
		if (CMyWorld::PlayersCount == 2 && i == 1) continue;
		const CMyCar& enemyCar = before.World.Cars[i];
		if (enemyCar.OiledTicks > 0) {
			enemyMaxOiledSpeedBefore = max(enemyMaxOiledSpeedBefore, before.World.Cars[i].Speed.Length());
		}
		enemyDurabilitySumBefore += enemyCar.Durability;
	}

	CState current(startWorld, 0, 0);
	const int simulationEnd = before.Tick;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove moves[4];
		size_t mi = 0;
		for (; mi < correctedPreviousMoveList.size(); mi++) {
			const CMoveWithDuration& m = correctedPreviousMoveList[mi];
			if (current.Tick >= m.Start && current.Tick < m.End) {
				moves[0] = m.Move;
				break;
			}
		}
		if (current.Tick == 0) {
			moves[0].Oil = true;
		}
		current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;
	}

	double enemyMaxOiledSpeedAfter = 0;
	double enemyDurabilitySumAfter = 0;
	for (int i = 1; i < CMyWorld::MaxCars; i++) {
		if (CMyWorld::PlayersCount == 2 && i == 1) continue;
		const CMyCar& enemyCar = current.World.Cars[i];
		if (enemyCar.OiledTicks > 0) {
			enemyMaxOiledSpeedAfter = max(enemyMaxOiledSpeedAfter, current.World.Cars[i].Speed.Length());
		}
		enemyDurabilitySumAfter += enemyCar.Durability;
	}

	//CLog::Instance().Stream() << "oil score = " << oiledEnemyMaxSpeedAfter << " - " << oiledEnemyMaxSpeedBefore << endl;
	double enemyDurabilityDif = enemyDurabilitySumBefore - enemyDurabilitySumAfter;
	double enemyMaxOiledSpeedDif = enemyMaxOiledSpeedAfter - enemyMaxOiledSpeedBefore;
	CLog::Instance().Stream() << "oil enemyDurabilityDif = " << enemyDurabilityDif << " = " << enemyDurabilitySumBefore << " - " << enemyDurabilitySumAfter << endl;
	CLog::Instance().Stream() << "oil enemyMaxOiledSpeedDif = " << enemyDurabilityDif << " = " << enemyMaxOiledSpeedAfter << " - " << enemyMaxOiledSpeedBefore << endl;

	if(enemyDurabilityDif > 0.1 || enemyMaxOiledSpeedDif > 20) {
		result.CurrentMove.Oil = true;
	}
}
