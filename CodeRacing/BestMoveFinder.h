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
	struct CResult {
		// �����.
		bool Success = false;
		// �� ��������, ������� ����� ����� ��������� ������.
		CMyMove CurrentMove;
		// ������������������ �������� - ������ �� ��� (��������, ��������� ��� �������� (�� ������������)).
		std::vector<std::pair<CMyMove, int>> MoveList;
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
	const CMyCar& car;
	const model::World& world;
	const model::Game& game;
	const std::vector<CMyTile>& tileRoute;
	const CSimulator& simulator;

};
