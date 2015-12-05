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

	struct CCollisionInfo {
		CVec2D Normal;
		CVec2D Point;
		double Depth;
	};

	void updateCar(const model::Move& move, int currentTick, CMyCar& car,
		bool& isOiled, bool& isBrake,
		CVec2D& lengthwiseUnitVector, CVec2D& accelerationDt) const;
	void updatePosition(
		CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed, CRotatedRect& rotatedRect, double& durability,
		CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
		double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
		double rotationAirFrictionFactorDt, double rotationFrictionFactorDt,
		bool passThroughWalls, double radius) const;

	void processCarWithWallsCollision(CVec2D& position, CVec2D& speed, double& angularSpeed,
		CRotatedRect& rotatedRect, double& collisionDeltaSpeed) const;
	void processCircleWithWallsCollision(CVec2D& position, CVec2D& speed, double& angularSpeed,
		double radius, double& collisionDeltaSpeed) const;

	bool findLineWithRotatedRectCollision(
		const CVec2D& point1A, const CVec2D& point2A,
		const CVec2D& position, const CRotatedRect& rotatedRect, double circumcircleRadius,
		CCollisionInfo& collisionInfo) const;
	bool findArcWithRotatedRectCollision(
		const CArc2D& arcB,
		const CVec2D& positionA, const CRotatedRect& rotatedRectA, double circumcircleRadiusA,
		CCollisionInfo& collisionInfo) const;
	bool findLineWithCircleCollision(
		const CVec2D& point1B, const CVec2D& point2B,
		const CVec2D& positionA, double radiusA,
		CCollisionInfo& collisionInfo) const;
	bool findArcWithCircleCollision(
		const CArc2D& arcB,
		const CVec2D& positionA, double radiusA,
		CCollisionInfo& collisionInfo) const;
	void resolveCollisionStatic(
		const CCollisionInfo& collisionInfo,
		CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRect, double& collisionDeltaSpeed,
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
