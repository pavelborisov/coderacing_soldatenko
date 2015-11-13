#include "Simulator.h"

CSimulator::CSimulator() : isInitialized(false)
{
}

void CSimulator::Initialize(const model::Game& _game)
{
	game = _game;
	isInitialized = true;
}

bool CSimulator::IsInitialized() const
{
	return isInitialized;
}

CVec2D CSimulator::Predict(const model::Car& car, const model::World& /*world*/, const model::Move& /*move*/) const
{
	return CVec2D(car.getX(), car.getY());
}