#pragma once

#include "model/Game.h"
#include "model/World.h"
#include "model/Move.h"
#include "MyCar.h"

class CSimulator {
public:
	CSimulator();
	void Initialize(const model::Game& game);
	bool IsInitialized() const;

	CMyCar Predict(const CMyCar& car, const model::World& world, const model::Move& move) const;

private:
	bool isInitialized;
	model::Game game;

};
