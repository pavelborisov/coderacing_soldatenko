#pragma once

#include <vector>
#include "model\Game.h"
#include "model\World.h"
#include "MyCar.h"
#include "MyMove.h"
#include "MyTile.h"
#include "Simulator.h"

class CBestMoveFinder {
public:
	// Действие и его время действия - первый и последний(не включительно) тики
	struct CMoveWithDuration{
		CMyMove Move;
		int Start = 0;
		int End = 0;
		CMoveWithDuration(const CMyMove& Move, int Start, int End) : Move(Move), Start(Start), End(End) {}
	};

	struct CResult {
		// Успех.
		bool Success = false;
		// То действие, которое лучше всего совершить сейчас.
		CMyMove CurrentMove;
		// Последовательность действий - массив из действий с их временами действия
		std::vector<CMoveWithDuration> MoveList;
		// Некоторая функция оценки всей последовательности действий.
		double Score;
	};

	CBestMoveFinder(
		const CMyCar& car,
		const model::World& world,
		const model::Game& game,
		const std::vector<CMyTile>& tileRoute,
		const CSimulator& simulator);
	
	CResult Process();

private:
	struct CState {
		CMyCar Car;
		int Tick = 0;
		int NextRouteIndex = 1;
		double RouteScore = 0;
		CState(const CMyCar& Car, int Tick, int NextRouteIndex, double RouteScore) : Car(Car), Tick(Tick), NextRouteIndex(NextRouteIndex), RouteScore(RouteScore) {}
	};

	const CMyCar& car;
	const model::World& world;
	const model::Game& game;
	const std::vector<CMyTile>& tileRoute;
	const CSimulator& simulator;

	int simulationTicks = 0;
	double bestScore = INT_MIN;
	std::vector<CMoveWithDuration> bestMoveList;
	std::vector<CState> stateCache;

	static const std::vector<std::pair<std::vector<CMyMove>, std::vector<int>>> allMovesWithLengths;
	static const int maxTick = 175;

	void processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList);
	double evaluate(const CState& state, bool brake) const;

};
