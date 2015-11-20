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
		// Успех.
		bool Success = false;
		// То действие, которое лучше всего совершить сейчас.
		CMyMove CurrentMove;
		// Последовательность действий - массив из пар (действие, последний тик действия (не включительно)).
		std::vector<std::pair<CMyMove, int>> MoveList;
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
	const CMyCar& car;
	const model::World& world;
	const model::Game& game;
	const std::vector<CMyTile>& tileRoute;
	const CSimulator& simulator;

};
