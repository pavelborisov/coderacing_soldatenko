#pragma once

#include <vector>
#include "model\Game.h"
#include "model\World.h"
#include "MyMove.h"
#include "MyTile.h"
#include "MyWorld.h"

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
		const CMyWorld& startWorld,
		const std::vector<CMyTile>& waypointTiles,
		const CBestMoveFinder::CResult& previousResult);
	CBestMoveFinder(
		const CMyWorld& startWorld,
		const std::vector<CMyTile>& waypointTiles,
		const CBestMoveFinder::CResult& previousResult,
		const CBestMoveFinder::CResult& allyResult,
		bool correctAllyResult);
	
	CResult Process();

private:
	struct CState {
		CMyWorld World;
		int Tick = 0;
		double RouteScore = 0;
		CState() {}
		CState(const CMyWorld& World, int Tick, double RouteScore) :
			World(World), Tick(Tick), RouteScore(RouteScore) {}
	};

	const CMyWorld& startWorld;
	const std::vector<CMyTile>& waypointTiles;
	std::vector<CMoveWithDuration> correctedPreviousMoveList;
	std::vector<CMoveWithDuration> allyMoveList;
	bool hasAlly;

	int simulationTicks = 0;
	double startScore = INT_MIN;
	double bestScore = INT_MIN;
	std::vector<CMoveWithDuration> bestMoveList;
	std::vector<CState> stateCache;

	static const std::vector<std::pair<std::vector<CMyMove>, std::vector<int>>> allMovesWithLengths;
	static const int maxTick = 140;

	void processPreviousMoveList();
	void processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList);
	void processRouteScore(CState& state, bool firstTickBrake);
	double evaluate(const CState& state) const;
	void postProcess(CResult& result);
	void postProcessShooting(const CState& before, CResult& result);
	void postProcessOil(const CState& before, CResult& result);
	void postProcessNitro(const CState& before, CResult& result);

};
