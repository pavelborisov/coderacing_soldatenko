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

CMyCar CSimulator::Predict(const CMyCar& car, const model::World& /*world*/, const model::Move& /*move*/) const
{
	CMyCar predictedCar(car);
	predictedCar.Position = car.Position + car.Speed;

	return predictedCar;
}
