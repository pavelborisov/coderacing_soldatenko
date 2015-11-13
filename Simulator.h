#pragma once

#include "model\Game.h"
#include "model\Car.h"
#include "model\World.h"
#include "model\Move.h"
#include "Vec2D.h"

class CSimulator {
public:
	CSimulator();
	void Initialize(const model::Game& game);
	bool IsInitialized() const;

	CVec2D Predict(const model::Car& car, const model::World& world, const model::Move& move) const;

private:
	bool isInitialized;
	model::Game game;

};
