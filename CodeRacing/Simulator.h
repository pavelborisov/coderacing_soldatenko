#pragma once

#include <vector>
#include "model/Game.h"
#include "model/Car.h"
#include "model/World.h"
#include "model/Move.h"
#include "GlobalPredictions.h"
#include "Arc2D.h"
#include "Line2D.h"
#include "MyCar.h"
#include "Vec3D.h"

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
		CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed, CRotatedRect& rotatedRect,
		CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
		double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
		double rotationAirFrictionFactorDt, double rotationFrictionFactorDt,
		bool passThroughWalls, double radius) const;

	void processWallsCollision(CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed,
		double radius, CRotatedRect& rotatedRect) const;

	bool findLineWithRotatedRectCollision(
		const CVec2D& point1A, const CVec2D& point2A,
		const CVec2D& position, const CRotatedRect& rotatedRect, double circumcircleRadius,
		CVec2D& collisionNormalB, CVec2D& collisionPoint, double& depth) const;
	bool findArcWithRotatedRectCollision(
		const CArc2D& arcA,
		const CVec2D& position, const CRotatedRect& rotatedRect, double circumcircleRadius,
		CVec2D& collisionNormalB, CVec2D& collisionPoint, double& depth) const;
	void resolveCollisionStatic(
		const CVec2D& collisionNormalB2D, const CVec2D& collisionPoint, double depth, 
		CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRect,
		double invertedMassA, double invertedAngularMassA,
		double momentumTransferFactorAB, double surfaceFrictionFactorAB) const;
	void resolveImpactStatic(
		const CVec3D& vectorAC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
		CVec2D& speedA, double& angularSpeedA,
		double invertedMassA, double invertedAngularMassA,
		double momentumTransferFactorAB) const;
	void resolveSurfaceFrictionStatic(
		const CVec3D& vectorAC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
		CVec2D& speedA, double& angularSpeedA,
		double invertedMassA, double invertedAngularMassA,
		double surfaceFrictionFactorAB) const;
	void pushBackBodiesStatic(
		const CVec2D& collisionNormalB2D, double depth,
		CVec2D& positionA, CRotatedRect& rotatedRect) const;

};
