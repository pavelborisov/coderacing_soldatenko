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

	// TODO: Отсечки по коллизиям.
	// TODO: Оценка на каждой симуляции или только на финальной?
	// TODO: Если действия не будут идти друг за другом впритык, а первое действие не будет начинать с нуля, то надо
	//       ещё добавить симуляцию "дефолтного" действия до начала текущего.
	// TODO: Для оценки - симулировать nextWP, считать карту расстояний от каждого wp до произвольных координат (с точностью до 1/4 клетки) - пустить волну от вп.
	// TODO: Если длина действия == 0, то проверять только первое действие из массива.
	for (CMyMove firstMove : firstMoveArray) {
		// Отдельная константа на случай если начало действия захочется сделать не сразу.
		const int firstStart = 0;
		// Кэш первого действия.
		int firstTick = 0; // На каком тике была закончена предыдущая симуляция первого действия.
		CMyCar firstCar = car; // Состояние, в которое мы пришли после предыдущей симуляции первого действия.
		double firstRouteScore = 0;
		int firstRouteIndex = 1;
		// Перебор длины действия. Должен быть в порядке увеличения длины, чтобы использовать кэш.
		for (int firstLength : firstLengthArray) {
			const int firstEnd = firstStart + firstLength;
			// Страхуемся от неправильного порядка в firstLengthArray.
			assert(firstTick <= firstEnd);
			// Продолжаем делать первое действие до его конца. При этом если конец был "удлинён", то
			// мы продолжим симуляцию с предыдущего результата.
			for (; firstTick < firstEnd; firstTick++) {
				firstCar = simulator.Predict(firstCar, world, firstMove.Convert());
				simulationTicks++;
				// Подсчёт очков за пройденную клетку маршрута. TODO: переделать
				if (CMyTile(firstCar.Position) == tileRoute[firstRouteIndex]) {
					firstRouteScore += 800;
					firstRouteIndex++;
				}
				if (firstCar.CollisionDetected) break; // Отсечка по коллизиям. TODO: не останавливаться, если коллизия была "мягкая"
			}

			// Второе действие.
			for (CMyMove secondMove : secondMoveArray) {
				const int secondStart = firstTick;
				// Кэш второго действия.
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
						// Подсчёт очков за пройденную клетку маршрута.
						if (CMyTile(secondCar.Position) == tileRoute[secondRouteIndex]) {
							secondRouteScore += 800;
							secondRouteIndex++;
						}
						if (secondCar.CollisionDetected) break; // Отсечка по коллизиям
					}

					// Третье действие.
					for (CMyMove thirdMove : thirdMoveArray) {
						const int thirdStart = secondTick;
						// Кэш третьего действия.
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
								// Подсчёт очков за пройденную клетку маршрута.
								if (CMyTile(thirdCar.Position) == tileRoute[thirdRouteIndex]) {
									thirdRouteScore += 800;
									thirdRouteIndex++;
								}
								if (thirdCar.CollisionDetected) break; // Отсечка по коллизиям
							}

							// Оценка итоговой позиции.
							CMyCar simCar = thirdCar; // Итоговое состояние.
							CMyTile simCarTile(thirdCar.Position); // В каком тайле оказались.
							CMyTile targetTile(tileRoute[thirdRouteIndex]); // Какой следующий тайл по маршруту.
							double score = thirdRouteScore; // Очки за все предыдущие полностью пройденные тайлы.
															// Очки за частично пройденный тайл, если мы оказались в соседнем тайле от целевого.
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
								score -= 0; // Пока не штрафуем.
							}
							// TODO: Дополнительный штраф за торможение?

							// Проверка наилучшего варианта.
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

	////////////////////////////////////// Собираем результат
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
