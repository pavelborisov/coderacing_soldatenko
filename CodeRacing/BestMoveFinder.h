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
		int nextWaypointIndex,
		const model::World& world,
		const model::Game& game,
		const std::vector<CMyTile>& waypointTiles,
		const CSimulator& simulator,
		const CBestMoveFinder::CResult& previousResult);
	
	CResult Process();

private:
	struct CState {
		CMyCar Car;
		int Tick = 0;
		int NextWaypointIndex = -1;
		double RouteScore = 0;
		std::vector<bool> PickedBonuses;
		CState(const CMyCar& Car, int Tick, int NextWaypointIndex, double RouteScore, size_t BonusesSize) :
			Car(Car), Tick(Tick), NextWaypointIndex(NextWaypointIndex), RouteScore(RouteScore)
		{
			PickedBonuses.assign(BonusesSize, false);
		}
	};

	const CMyCar& car;
	int nextWaypointIndex;
	const model::World& world;
	const model::Game& game;
	const std::vector<CMyTile>& waypointTiles;
	const CSimulator& simulator;
	std::vector<CMoveWithDuration> correctedPreviousMoveList;

	std::vector<model::Bonus> bonuses;
	std::vector<CVec2D> bonusPositions;

	int simulationTicks = 0;
	double bestScore = INT_MIN;
	std::vector<CMoveWithDuration> bestMoveList;
	std::vector<CState> stateCache;

	static const std::vector<std::pair<std::vector<CMyMove>, std::vector<int>>> allMovesWithLengths;
	static const int maxTick = 125;

	void processPreviousMoveList();
	void processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList);
	void processRouteScore(CState& state, bool firstTickBrake);
	void processBonus(CState& state);
	double evaluate(const CState& state) const;

};
