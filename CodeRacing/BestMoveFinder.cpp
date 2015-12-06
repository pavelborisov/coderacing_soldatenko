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
static const int carSimulationSubticksCount = 2;

const vector<pair<vector<CMyMove>, vector<int>>> CBestMoveFinder::allMovesWithLengths = {
	// ������ ��������� ��������
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 },{ 1, 1 },{ -1, 1 } },
		{ 0, 5, 10, 20, 40 }
	},
	// ������ ��������� ��������
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 },{ 1, 1 },{ -1, 1 } },
		{ 0, 40 }
	},
	// ������ ��������� ��������
	{
		{ { 1, 0 },{ -1, 0 } },
		{ 0, 40 }
	},
	// �������� �������� - ������ ����� �� ������ (���� �� ������ � maxTick)
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
	CWorldSimulator::Instance().SetPrecision(carSimulationSubticksCount);
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
	//CMyCar simCar = car;
	//const int simulationEnd = bestMoveList.back().End;
	//for (int tick = 0; tick < simulationEnd; tick++) {
	//	model::Move simMove;
	//	for (const auto& m : bestMoveList) {
	//		if (tick >= m.Start && tick < m.End) {
	//			simMove = m.Move.Convert();
	//		}
	//	}
	//	simCar = simulator.Predict(simCar, simMove, tick);
	//	CDrawPlugin::Instance().FillCircle(simCar.Position.X, simCar.Position.Y, 5, 0x0000FF);
	//}
	//const double halfHeight = game.getCarHeight() / 2;
	//const double halfWidth = game.getCarWidth() / 2;
	//for (const auto& corner : simCar.RotatedRect.Corners) {
	//	CDrawPlugin::Instance().FillCircle(corner.X, corner.Y, 5, 0x00FFFF);
	//}

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
		current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
		simulationTicks++;

		processRouteScore(current, current.Tick == 0 && moves[0].Brake == true);

		// ������� �� ���������. TODO: �� ���������������, ���� �������� ���� "������"
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
		stateCache.push_back({ startWorld, 0, 0 });
	}

	CMyMove moves[4];
	for (const CMyMove& move : moveArray) {
		moves[0] = move;
		CState current = stateCache.back();
		const int start = current.Tick;
		for (int length : lengthsArray) {
			if (length == 0 && move != moveArray[0]) {
				// ���� � �������� ����� == 0, �� ��� �� ����� ������� ��������� ��������. ������ ������ ������� �� �������.
				continue;
			}
			const int end = min(maxTick, start + length);
			assert(current.Tick <= end); // �������� ����������� ������� � ������� lenghtsArray
			// ���������� ������������ � ���� �����, ������ ��������� � ���������� ���.
			for (; current.Tick < end; current.Tick++) {
				current.World = CWorldSimulator::Instance().Simulate(current.World, moves);
				simulationTicks++;

				processRouteScore(current, current.Tick == 0 && move.Brake == 1);

				// ������� �� ���������. TODO: �� ���������������, ���� �������� ���� "������"
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
						// ��������� �� �����
						break;
					}
				//}
			}

			if (!lastMove) {
				// ��������� ��������� ���������� ��������.
				vector<CMoveWithDuration> moveList = prevMoveList;
				moveList.push_back({ move, start, end });
				stateCache.push_back(current);
				processMoveIndex(moveIndex + 1, moveList);
				stateCache.pop_back();
			}

			if (end == maxTick) {
				// ��� ������ ������ ����������� ����� ��������, �.�. �� ��� ���� ��������� �� ����������� �� ������������� ���-�� �����.
				break;
			}
		}
	}
}

void CBestMoveFinder::processRouteScore(CState& state, bool firstTickBrake)
{
	// ������� ����� �� ���������� ������ ��������.
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

	// ����� �� ���������� � ����� ������.
	if (firstTickBrake) {
		state.RouteScore -= 200;
	}

	// ����� �� ����� � ����. ����� �� ��������.
	if (car.OiledTicks == 59 && car.Speed.Length() > 12) {
		state.RouteScore -= (car.Speed.Length() - 12) * 100;
	}

	// ������ �������
	processBonus(state);
}

void CBestMoveFinder::processBonus(CState& state)
{
	const CMyCar& startCar = startWorld.Cars[0];
	const CMyCar& car = state.World.Cars[0];
	const double durabilityDifPositive = max(0.0, car.Durability - startCar.Durability);
	const int ammoDifPositive = max(0, car.ProjectilesCount - startCar.ProjectilesCount);
	const int nitroDifPositive = max(0, car.NitroCount - startCar.NitroCount);
	const int oilDifPositive = max(0, car.OilCount - startCar.OilCount);
	const int scoreDif = state.World.Players[0].Score - startWorld.Players[0].Score;

	state.RouteScore += durabilityDifPositive * 1500;
	state.RouteScore += ammoDifPositive * 400;
	state.RouteScore += nitroDifPositive * 1000;
	state.RouteScore += oilDifPositive * 200;
	state.RouteScore += scoreDif * 15;
}

double CBestMoveFinder::evaluate(const CState& state) const
{
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

	// ����� �� ������ ��.
	if (car.Durability < 1e-7) {
		score += -1000000;
	} else {
		// == -20000 ��� durability == 1 � == -200000 ��� durability == 0.01
		score += -20000 * sqrt(1 / car.Durability);
	}

	return score;
}
