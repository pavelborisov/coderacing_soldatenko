#include "BestMoveFinder.h"

#include <assert.h>
#include "DrawPlugin.h"
#include "Log.h"

using namespace std;

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
	static int totalSimulationTicks = 0;
	int simulationTicks = 0;

	double bestScore = INT_MIN;
	CMyCar bestSimCar;
	CMyMove bestFirstMove;
	int bestFirstTick = 0;
	CMyMove bestSecondMove;
	int bestSecondTick = 0;
	CMyMove bestThirdMove;
	int bestThirdTick = 0;

	CMyMove firstMoveArray[] = { { 0, 0 },{ -1, 0 },{ 1, 0 } };
	int firstLengthArray[] = { 0, 5, 10, 20, 30 };
	CMyMove secondMoveArray[] = { { 0, 0 },{ 0, 1 } };
	int secondLengthArray[] = { 0, 30 };
	CMyMove thirdMoveArray[] = { { 0, 0 },{ -1, 0 },{ 1, 0 } };
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
				firstCar = simulator.Predict(firstCar, world, firstMove.Convert());
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
						secondCar = simulator.Predict(secondCar, world, secondMove.Convert());
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
								thirdCar = simulator.Predict(thirdCar, world, thirdMove.Convert());
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
	CLog::Instance().Log(simulationTicks, "SimulationTicks");
	CLog::Instance().Log(totalSimulationTicks, "TotalSimulationTicks");

	////////////////////////////////////////// draw best
	CMyCar simCar = car;
	CDrawPlugin::Instance().SetColor(0, 0, 255);
	for (int tick = 0; tick < bestThirdTick; tick++) {
		model::Move simMove;

		if (tick < bestFirstTick) {
			simMove = bestFirstMove.Convert();
		} else if (tick < bestSecondTick) {
			simMove = bestSecondMove.Convert();
		} else if (tick < bestThirdTick) {
			simMove = bestThirdMove.Convert();
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
	if (bestFirstTick > 0) {
		result.CurrentMove = bestFirstMove;
		result.Success = true;
	} else if (bestSecondTick > 0) {
		result.CurrentMove = bestSecondMove;
		result.Success = true;
	} else if (bestThirdTick > 0) {
		result.CurrentMove = bestThirdMove;
		result.Success = true;
	}
	result.Score = bestScore;
	result.MoveList = { {bestFirstMove, bestFirstTick}, {bestSecondMove, bestSecondTick}, {bestThirdMove, bestThirdTick} };
	return result;
}
