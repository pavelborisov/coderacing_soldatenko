#include "BestMoveFinder.h"

#include <assert.h>
#include <algorithm>
#include "DrawPlugin.h"
#include "Log.h"

using namespace std;

// TODO: �������� �������� "���", ������� ������ ���� �����.
const vector<pair<vector<CMyMove>, vector<int>>> CBestMoveFinder::allMovesWithLengths = {
	// ������ ��������� �������� - ���� ����� ��� � ����������.
	{
		{ { 0, 0 },{ 1, 0 },{ -1, 0 } },
		{ 0, 5, 10, 20, 30 }
	},
	// ������ ��������� �������� - ��������� ��� ���.
	{
		{ { 0, 0 },{ 0, 1 } },
		{ 0, 30 }
	},
	// ������ ��������� �������� - ���� ����� ��� � ����������.
	{
		{ { 1, 0 },{ -1, 0 } },
		{ 0, 10, 25, 50 }
	},
	// �������� �������� - ������ ����� �� ������ (���� �� ������ � maxTick)
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
		const CSimulator& simulator) :
	car(car),
	world(world),
	game(game),
	tileRoute(tileRoute),
	simulator(simulator)
{
}

CBestMoveFinder::CResult CBestMoveFinder::Process()
{
	simulationTicks = 0;
	bestScore = INT_MIN;
	bestMoveList.clear();
	stateCache.push_back({car, 0, 1, 0});

	processMoveIndex(0, vector<CMoveWithDuration>());
	assert(bestMoveList.size() == allMovesWithLengths.size());

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
	vector<CVec2D> carCorners(4);
	carCorners[0] = CVec2D(halfWidth, halfHeight);
	carCorners[1] = CVec2D(halfWidth, -halfHeight);
	carCorners[2] = CVec2D(-halfWidth, -halfHeight);
	carCorners[3] = CVec2D(-halfWidth, halfHeight);
	CDrawPlugin::Instance().SetColor(0, 255, 255);
	for (auto& corner : carCorners) {
		corner.Rotate(simCar.Angle);
		corner += simCar.Position;
		CDrawPlugin::Instance().FillCircle(corner, 5);
	}

	////////////////////////////////////// �������� ���������
	CResult result;
	result.MoveList = bestMoveList;
	for (const auto& m : bestMoveList) {
		if (m.End > 0) {
			result.CurrentMove = m.Move;
			break;
		}
	}
	result.Score = bestScore;
	return result;
}

void CBestMoveFinder::processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList)
{
	const bool lastMove = moveIndex == allMovesWithLengths.size() - 1;
	const vector<CMyMove>& moveArray = allMovesWithLengths[moveIndex].first;
	const vector<int>& lengthsArray = allMovesWithLengths[moveIndex].second;

	for (const CMyMove& move : moveArray) {
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
				current.Car = simulator.Predict(current.Car, world, move.Convert());
				simulationTicks++;
				// ������� ����� �� ���������� ������ ��������. TODO: ����������
				if (CMyTile(current.Car.Position) == tileRoute[current.NextRouteIndex]) {
					current.RouteScore += 800;
					current.NextRouteIndex = (current.NextRouteIndex + 1) % tileRoute.size();
				}
				// ������� �� ���������. TODO: �� ���������������, ���� �������� ���� "������"
				if (current.Car.CollisionDetected) {
					break;
				}
				if (lastMove) {
					vector<CMoveWithDuration> moveList = prevMoveList;
					moveList.push_back({ move, start, current.Tick });
					double score = evaluate(current, moveList);
					if (score > bestScore) {
						bestScore = score;
						bestMoveList = moveList;
					}
				}
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

double CBestMoveFinder::evaluate(const CState& state, const std::vector<CMoveWithDuration>& /*moveList*/) const
{
	CMyTile carTile(state.Car.Position); // � ����� ����� ���������.
	CMyTile targetTile(tileRoute[state.NextRouteIndex]); // ����� ��������� ���� �� ��������.
	double score = state.RouteScore; // ���� �� ��� ���������� ��������� ���������� ����� �� ��������.
	if (carTile.X == targetTile.X + 1) {
		score += (carTile.X + 1) * 800 - state.Car.Position.X;
	} else if (carTile.X == targetTile.X - 1) {
		score += state.Car.Position.X - (carTile.X) * 800;
	} else if (carTile.Y == targetTile.Y + 1) {
		score += (carTile.Y + 1) * 800 - state.Car.Position.Y;
	} else if (carTile.Y == targetTile.Y - 1) {
		score += state.Car.Position.Y - (carTile.Y) * 800;
	} else {
		// ���-�� ������ �� ���������.
		//score -= (state.Car.Position - carTile.ToVec()).Length();
		score -= 0; // ���� �� ��������.
	}
	// �������������� ����� �� ��, ��� �������� ������ ���������.
	//if (moveList[0].Move.Brake == 1) {
	//	score -= 800;
	//}
	return score;
}
