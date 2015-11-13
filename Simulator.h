#pragma once

#include <vector>
#include "model/Game.h"
#include "model/Car.h"
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
	// ��������� ����������� (����/�����).
	std::vector<double> forwardAccelByType;
	std::vector<double> rearAccelByType;
	// ������������ ������, � ������ dTime.
	double lengthwiseFrictionFactorDt;
	double crosswiseFrictionFactorDt;
	double rotationFrictionFactorDt;
	double movementAirFrictionFactorDt;
	double rotationAirFrictionFactorDt;
	// ������ ����������� ��������� �� game, ������� ���� ���������.
	double powerChangePerTick;
	double wheelTurnChangePerTick;
	double angularSpeedFactor;

};
