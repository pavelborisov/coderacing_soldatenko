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
		std::vector<bool> PickedBonuses;
		CState(const CMyCar& Car, int Tick, int NextRouteIndex, double RouteScore, size_t BonusesSize) :
			Car(Car), Tick(Tick), NextRouteIndex(NextRouteIndex), RouteScore(RouteScore)
		{
			PickedBonuses.assign(BonusesSize, false);
		}
	};

	const CMyCar& car;
	const model::World& world;
	const model::Game& game;
	const std::vector<CMyTile>& tileRoute;
	const CSimulator& simulator;

	std::vector<model::Bonus> bonuses;
	std::vector<CVec2D> bonusPositions;

	int simulationTicks = 0;
	double bestScore = INT_MIN;
	std::vector<CMoveWithDuration> bestMoveList;
	std::vector<CState> stateCache;

	static const std::vector<std::pair<std::vector<CMyMove>, std::vector<int>>> allMovesWithLengths;
	static const int maxTick = 175;

	void processMoveIndex(size_t moveIndex, const std::vector<CMoveWithDuration>& prevMoveList);
	void processBonus(CState& state);
	double evaluate(const CState& state) const;

};
