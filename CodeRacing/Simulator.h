#pragma once

#include <vector>
#include "model/Game.h"
#include "model/Car.h"
#include "model/World.h"
#include "model/Move.h"
#include "GlobalPredictions.h"
#include "MyCar.h"

class CSimulator {
public:
	CSimulator();
	void Initialize(const model::Game& game);
	bool IsInitialized() const;

	CMyCar Predict(const CMyCar& startCar, const model::Move& move, int currentTick) const;
	CMyWasher Predict(const CMyWasher& startWasher, int currentTick) const;
	CMyTire Predict(const CMyTire& startTire, int currentTick) const;

private:
	bool isInitialized;
	model::Game game;
	// Ускорения автомобилей (сила/масса).
	std::vector<double> forwardAccelByType;
	std::vector<double> rearAccelByType;
	// Коэффициенты трения, с учётом dTime.
	double carLengthwiseFrictionFactorDt;
	double carCrosswiseFrictionFactorDt;
	double carRotationFrictionFactorDt;
	double carMovementAirFrictionFactorDt;
	double carRotationAirFrictionFactorDt;

	void updateCar(const model::Move& move, int currentTick, CMyCar& car,
		bool& isOiled, bool& isBrake,
		CVec2D& lengthwiseUnitVector, CVec2D& accelerationDt) const;
	void updatePosition(
		CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed,
		CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
		double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
		double rotationAirFrictionFactorDt, double rotationFrictionFactorDt,
		bool passThroughWalls, double radius, CRotatedRect& rotatedRect ) const;

	void processWallCollision(CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed,
		double radius, CRotatedRect& rotatedRect) const;

};
