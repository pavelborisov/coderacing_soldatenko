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
static const int minEvaluateTick = 40;

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

static CMyMove findMove(int tick, const vector<CBestMoveFinder::CMoveWithDuration>& movesWithDuration)
{
	for (size_t mi = 0; mi < movesWithDuration.size(); mi++) {
		const CBestMoveFinder::CMoveWithDuration& m = movesWithDuration[mi];
		if (tick >= m.Start && tick < m.End) {
			return m.Move;
		}
	}
	return CMyMove();
}

static CMyMove findMove(const CMyWorld& world, int carId)
{
	CMyMove m;
	m.Engine = 1;
	m.Turn = world.Cars[carId].WheelTurn;
	return m;
}

CBestMoveFinder::CBestMoveFinder(
	const CMyWorld& startWorld,
	const std::vector<CMyTile>& waypointTiles,
	const CBestMoveFinder::CResult& previousResult,
	TMode mode) :
	startWorld(startWorld),
	waypointTiles(waypointTiles),
	hasAlly(false),
	mode(mode)
{
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

CBestMoveFinder::CBestMoveFinder(
	const CMyWorld& startWorld,
	const std::vector<CMyTile>& waypointTiles,
	const CBestMoveFinder::CResult& previousResult,
	const CBestMoveFinder::CResult& allyResult,
	bool correctAllyResult,
	TMode mode) :
	CBestMoveFinder(startWorld, waypointTiles, previousResult, mode)
{
	allyMoveList = allyResult.MoveList;
	hasAlly = true;

	if (correctAllyResult) {
		for (int i = allyMoveList.size() - 1; i >= 0; i--) {
			allyMoveList[i].Start--;
			if (allyMoveList[i].End < maxTick) {
				allyMoveList[i].End--;
			}
			if (allyMoveList[i].End < 0) {
				allyMoveList.erase(allyMoveList.begin() + i);
			}
		}
	}
}

CBestMoveFinder::CResult CBestMoveFinder::Process(bool checkRear)
{
	simulationTicks = 0;
	bestScore = INT_MIN;
	bestMoveList.clear();

	CState start(startWorld, 0, 0);
	startScore = evaluate(start);

	CResult result;
	processPreviousMoveList();
	if (mode != M_OnlyPrevious) {
		processMoveIndex(0, vector<CMoveWithDuration>(), checkRear);
	}
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
	if (mode != M_OnlyPrevious) {
		postProcess(result);
	}

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
		moves[0] = findMove(tick, bestMoveList);
		if (hasAlly) moves[1] = findMove(tick, allyMoveList); else moves[1] = findMove(simWorld, 1);
		moves[2] = findMove(simWorld, 2);
		moves[3] = findMove(simWorld, 3);
		setSimulatorMode(tick);
		CWorldSimulator::Instance().Simulate(simWorld, moves);
		//int color = min(200, 100 + tick);
		//simWorld.Draw(0xFF00FF + 0x000100 * color);
		CDrawPlugin::Instance().FillCircle(simWorld.Cars[0].Position.X, simWorld.Cars[0].Position.Y, 5, 0x0000FF);
		CDrawPlugin::Instance().FillCircle(simWorld.Cars[1].Position.X, simWorld.Cars[1].Position.Y, 5, 0x00FF00);
		CDrawPlugin::Instance().FillCircle(simWorld.Cars[2].Position.X, simWorld.Cars[2].Position.Y, 5, 0xFF0000);
		CDrawPlugin::Instance().FillCircle(simWorld.Cars[3].Position.X, simWorld.Cars[3].Position.Y, 5, 0xFF0000);
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
		moves[0] = findMove(current.Tick, correctedPreviousMoveList);
		if (hasAlly) moves[1] = findMove(current.Tick, allyMoveList); else moves[1] = findMove(current.World, 1);
		moves[2] = findMove(current.World, 2);
		moves[3] = findMove(current.World, 3);
		setSimulatorMode(current.Tick);
		CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;

		processRouteScore(current, current.Tick == 0 && moves[0].Brake == true);

		// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
		if (current.World.Cars[0].CollisionsDetected > maxCollisionsDetected) {
			break;
		}

		if (current.Tick >= minEvaluateTick) {
			//double score = evaluate(current);
			double score = evaluate(current) - startScore;
			if (score > bestScore) {
				bestScore = score;
				bestMoveList = correctedPreviousMoveList;
				// Обрезка
				size_t mi = 0;
				for (; mi < bestMoveList.size(); mi++) {
					const CMoveWithDuration& m = bestMoveList[mi];
					if (current.Tick >= m.Start && current.Tick < m.End) {
						break;
					}
				}
				bestMoveList[mi].End = current.Tick + 1;
				bestMoveList.erase(bestMoveList.begin() + mi + 1, bestMoveList.end());
			}
		}
	}
}

void CBestMoveFinder::processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList, bool checkRear)
{
	vector<CMyMove> moveArray;
	vector<int> lengthsArray;
	bool lastMove;

	lastMove = moveIndex == 3;
	double engine = checkRear ? -1 : 1;
	if (moveIndex == 0) {
		moveArray.push_back({ 0, 0, engine });
		lengthsArray.push_back(0);
		if (chance(50)) {
			moveArray.push_back({ 1, 0, engine });
			moveArray.push_back({ 1, 1, engine });
			//if (chance(50)) moveArray.push_back({ 1, 1, engine });
		} else {
			moveArray.push_back({ -1, 0, engine });
			moveArray.push_back({ -1, 1, engine });
			//if (chance(50)) moveArray.push_back({ -1, 1, engine });
		}
		lengthsArray.push_back(uniform(4, 12));
		lengthsArray.push_back(uniform(12, 30));
		lengthsArray.push_back(uniform(30, 50));
		//lengthsArray.push_back(uniform(50, 70));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 1) {
		moveArray.push_back({ 0, 0, engine });
		lengthsArray.push_back(0);
		if (chance(50)) {
			moveArray.push_back({ 1, 0, engine });
			//moveArray.push_back({ 1, 1, engine });
			if (chance(50)) moveArray.push_back({ 1, 1, engine });
		} else {
			moveArray.push_back({ -1, 0, engine });
			//moveArray.push_back({ -1, 1, engine });
			if (chance(50)) moveArray.push_back({ -1, 1, engine });
		}
		lengthsArray.push_back(uniform(30, 50));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 2) {
		moveArray.push_back({ 0, 0, engine });
		lengthsArray.push_back(0);
		if (chance(50)) {
			moveArray.push_back({ 1, 0, engine });
		} else {
			moveArray.push_back({ -1, 0, engine });
		}
		lengthsArray.push_back(uniform(30, 50));
		sort(lengthsArray.begin(), lengthsArray.end());
	} else if (moveIndex == 3) {
		moveArray.push_back({ 0, 0, engine });
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
				if (hasAlly) moves[1] = findMove(current.Tick, allyMoveList); else moves[1] = findMove(current.World, 1);
				moves[2] = findMove(current.World, 2);
				moves[3] = findMove(current.World, 3);
				setSimulatorMode(current.Tick);
				CWorldSimulator::Instance().Simulate(current.World, moves);
				simulationTicks++;

				processRouteScore(current, current.Tick == 0 && move.Brake == 1);

				// Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
				if (current.World.Cars[0].CollisionsDetected > maxCollisionsDetected) {
					break;
				}

				if(current.Tick > minEvaluateTick) {
					//double score = evaluate(current);
					double score = evaluate(current) - startScore;
					if (score > bestScore) {
						bestScore = score;
						bestMoveList = prevMoveList;
						bestMoveList.push_back({ move, start, current.Tick + 1 });
					//} else if (score - startScore < veryBadScoreDif) {
					} else if (score < veryBadScoreDif) {
						// Отсечение по очкам
						break;
					}
				}
			}

			if (!lastMove) {
				// Запускаем обработку следующего действия.
				vector<CMoveWithDuration> moveList = prevMoveList;
				moveList.push_back({ move, start, end });
				stateCache.push_back(current);
				processMoveIndex(moveIndex + 1, moveList, checkRear);
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

	// Штраф за торможение в самом начале.
	if (firstTickBrake) {
		state.RouteScore -= 200;
	}

	// Штраф за въезд в лужу. Растёт от скорости.
	if (car.OiledTicks == 59 && car.Speed.Length() > 12) {
		state.RouteScore -= (car.Speed.Length() - 12) * 100;
	}
}

double CBestMoveFinder::evaluate(const CState& state) const
{
	const CMyCar& startCar = startWorld.Cars[0];
	const CMyCar& car = state.World.Cars[0];
	bool rearIsBetterNotUsed = false;
	double dist = CWaypointDistanceMap::Instance().Query(car, rearIsBetterNotUsed);
	double score = state.RouteScore;
	score -= dist;

	if (CMyWorld::PlayersCount == 2) {
		bool notUsed;
		const CMyCar& allyCar = state.World.Cars[1];
		double allyDist = CWaypointDistanceMap::Instance().Query(allyCar, notUsed);
		const CMyCar& enemy1Car = state.World.Cars[2];
		double enemy1Dist = CWaypointDistanceMap::Instance().Query(enemy1Car, notUsed);
		const CMyCar& enemy2Car = state.World.Cars[2];
		double enemy2Dist = CWaypointDistanceMap::Instance().Query(enemy2Car, notUsed);
		score += enemy1Dist;
		score += enemy2Dist;
		score -= allyDist;

		if (enemy1Dist < dist) {
			score -= 1000;
		}
		if (enemy2Dist < dist) {
			score -= 1000;
		}

		const CMyCar& enemy1StartCar = startWorld.Cars[2];
		const CMyCar& enemy2StartCar = startWorld.Cars[2];
		const double enemy1DurabilityDif = enemy1Car.Durability - enemy1StartCar.Durability;
		const double enemy2DurabilityDif = enemy2Car.Durability - enemy2StartCar.Durability;
		if (enemy1DurabilityDif < 0) {
			if (enemy1Car.Durability < 1e-7) {
				score += 10000;
			} else {
				score += 1000 * sqrt(1 / enemy1Car.Durability);
			}
		}
		if (enemy2DurabilityDif < 0) {
			if (enemy2Car.Durability < 1e-7) {
				score += 10000;
			} else {
				score += 1000 * sqrt(1 / enemy2Car.Durability);
			}
		}
	}

	// Штраф за потерю хп.
	const double durabilityDif = car.Durability - startCar.Durability;
	if (durabilityDif < 0) {
		if (car.Durability < 1e-7) {
			score += -1000000;
		} else {
			// == -5000 при durability == 1 и == -50000 при durability == 0.01
			score += -5000 * sqrt(1 / car.Durability);
		}
	}

	const double durabilityDifPositive = max(0.0, durabilityDif);
	const int ammoDifPositive = max(0, car.ProjectilesCount - startCar.ProjectilesCount);
	const int nitroDifPositive = max(0, car.NitroCount - startCar.NitroCount);
	const int oilDifPositive = max(0, car.OilCount - startCar.OilCount);
	const int moneyDifPositive = max(0, car.MoneyCount - startCar.MoneyCount);
	//int scoreDif = state.World.Players[0].Score - startWorld.Players[0].Score;

	score += durabilityDifPositive * durabilityDifPositive * 3000;
	score += ammoDifPositive * 400;
	score += nitroDifPositive * 800;
	score += oilDifPositive * 200;
	score += moneyDifPositive * 1200;
	//score += scoreDif * 15;

	return score;
}

void CBestMoveFinder::postProcess(CResult& result)
{
	CWorldSimulator::Instance().SetOptions(false, false, false);
	CWorldSimulator::Instance().SetPrecision(2);
	CState current(startWorld, 0, 0);
	const int simulationEndShort = 40;
	const int simulationEndLong = 80;
	CState beforeShort;
	CState beforeLong;
	for (current.Tick = 0; current.Tick < simulationEndLong; current.Tick++) {
		CMyMove moves[4];
		moves[0] = findMove(current.Tick, bestMoveList);
		if (hasAlly) moves[1] = findMove(current.Tick, allyMoveList); else moves[1] = findMove(current.World, 1);
		moves[2] = findMove(current.World, 2);
		moves[3] = findMove(current.World, 3);
		CWorldSimulator::Instance().Simulate(current.World, moves);
		if (current.Tick == simulationEndShort - 1) {
			beforeShort = current;
		}
		if (current.Tick == simulationEndLong - 1) {
			beforeLong = current;
		}
		simulationTicks++;
	}

	postProcessShooting(beforeShort, result);
	postProcessOil(beforeShort, result);
	postProcessNitro(beforeLong, result);
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
		moves[0] = findMove(current.Tick, bestMoveList);
		if (hasAlly) moves[1] = findMove(current.Tick, allyMoveList); moves[1] = findMove(current.World, 1);
		moves[2] = findMove(current.World, 2);
		moves[3] = findMove(current.World, 3);
		if (current.Tick == 0) {
			moves[0].Shoot = true;
		}
		CWorldSimulator::Instance().Simulate(current.World, moves);
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
		if (!enemyCar.IsValid()) continue;
		if (enemyCar.OiledTicks > 0) {
			enemyMaxOiledSpeedBefore = max(enemyMaxOiledSpeedBefore, before.World.Cars[i].Speed.Length());
		}
		enemyDurabilitySumBefore += enemyCar.Durability;
	}

	CState current(startWorld, 0, 0);
	const int simulationEnd = before.Tick;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove moves[4];
		moves[0] = findMove(current.Tick, bestMoveList);
		if (hasAlly) moves[1] = findMove(current.Tick, allyMoveList); else moves[1] = findMove(current.World, 1);
		moves[2] = findMove(current.World, 2);
		moves[3] = findMove(current.World, 3);
		if (current.Tick == 0) {
			moves[0].Oil = true;
		}
		CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;
	}

	double enemyMaxOiledSpeedAfter = 0;
	double enemyDurabilitySumAfter = 0;
	for (int i = 1; i < CMyWorld::MaxCars; i++) {
		if (CMyWorld::PlayersCount == 2 && i == 1) continue;
		const CMyCar& enemyCar = current.World.Cars[i];
		if (!enemyCar.IsValid()) continue;
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

void CBestMoveFinder::postProcessNitro(const CState& before, CResult& result)
{
	const auto& car = startWorld.Cars[0];
	if (car.NitroCount == 0 || car.NitroCooldown > 0 || result.CurrentMove.Brake == true || result.CurrentMove.Engine < 0) {
		return;
	}

	double scoreBefore = evaluate(before);

	CState current(startWorld, 0, 0);
	const int simulationEnd = before.Tick;
	int brakeTicks = 0;
	for (current.Tick = 0; current.Tick < simulationEnd; current.Tick++) {
		CMyMove moves[4];
		moves[0] = findMove(current.Tick, bestMoveList);
		if (hasAlly) moves[1] = findMove(current.Tick, allyMoveList); else moves[1] = findMove(current.World, 1);
		moves[2] = findMove(current.World, 2);
		moves[3] = findMove(current.World, 3);
		if (current.Tick == 0) {
			moves[0].Nitro = true;
		}
		if (moves[0].Brake) {
			brakeTicks++;
		}
		CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;
	}

	if (brakeTicks > 0) {
		return;
	}


	// TODO: Нитро на старте лучше почти всегда нажимать. Но в середине игры, наверное, надо как-то оставлять его на обгоны...
	double scoreAfter = evaluate(current);

	static const double nitroScoreDifThreshold = 200;
	if (scoreAfter - scoreBefore > nitroScoreDifThreshold) {
		result.CurrentMove.Nitro = true;
	}
}
