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

	CMyCar Predict(const CMyCar& car, const model::World& world, const model::Move& move, int currentTick) const;
	CMyTire Predict(const CMyTire& tire, const model::World& world, int currentTick) const;

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
	void updateCarPosition(
		CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed,
		CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
		double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
		double rotationAirFrictionFactorDt, double rotationFrictionFactorDt) const;
	void updateCirclePosition(
		CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed,
		double movementAirFrictionFactorDt, double movementFrictionFactorDt,
		double rotationAirFrictionFactorDt, double rotationFrictionFactorDt) const;

	void processWallCollision(const CMyCar& startCar, CMyCar& car) const;

};
