#pragma once

#include <vector>
#include "model\Game.h"
#include "model\World.h"
#include "MyMove.h"
#include "MyTile.h"
#include "MyWorld.h"

class CBestMoveFinder {
public:
	enum TMode {
		M_Normal = 0,
		M_OnlyPrevious
	};

	// �������� � ��� ����� �������� - ������ � ���������(�� ������������) ����
	struct CMoveWithDuration{
		CMyMove Move;
		int Start = 0;
		int End = 0;
		CMoveWithDuration(const CMyMove& Move, int Start, int End) : Move(Move), Start(Start), End(End) {}
	};

	struct CResult {
		// �����.
		bool Success = false;
		// �� ��������, ������� ����� ����� ��������� ������.
		CMyMove CurrentMove;
		// ������������������ �������� - ������ �� �������� � �� ��������� ��������
		std::vector<CMoveWithDuration> MoveList;
		// ��������� ������� ������ ���� ������������������ ��������.
		double Score;
	};

	CBestMoveFinder(
		const CMyWorld& startWorld,
		const std::vector<CMyTile>& waypointTiles,
		const CBestMoveFinder::CResult& previousResult,
		TMode mode);
	CBestMoveFinder(
		const CMyWorld& startWorld,
		const std::vector<CMyTile>& waypointTiles,
		const CBestMoveFinder::CResult& previousResult,
		const CBestMoveFinder::CResult& allyResult,
		bool correctAllyResult,
		TMode mode);
	
	CResult Process(bool checkRear);

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
	TMode mode;

	int simulationTicks = 0;
	double startScore = INT_MIN;
	double bestScore = INT_MIN;
	std::vector<CMoveWithDuration> bestMoveList;
	std::vector<CState> stateCache;

	static const int maxTick = 120;

	void processPreviousMoveList();
	void processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList, bool checkRear);
	void processRouteScore(CState& state, bool firstTickBrake);
	double evaluate(const CState& state) const;
	void postProcess(CResult& result);
	void postProcessShooting(const CState& before, CResult& result);
	void postProcessOil(const CState& before, CResult& result);
	void postProcessNitro(const CState& before, CResult& result);

};
