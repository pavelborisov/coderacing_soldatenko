#pragma once

#include "model\Game.h"
#include "MyMove.h"
#include "MyWorld.h"
#include "Vec3D.h"

class CWorldSimulator {
public:
	static CWorldSimulator& Instance()
	{
		static CWorldSimulator singleInstance;
		return singleInstance;
	}

	void SetGame(const model::Game& game);
	void SetPrecision(int subtickCount);

	CMyWorld Simulate(const CMyWorld& startWorld, const CMyMove moves[CMyWorld::MaxCars]) const;

private:
	struct CCarInfo {
		bool IsOiled = false;
		bool IsBrake = false;
		CVec2D LengthwiseUnitVector;
		CVec2D CrosswiseUnitVector;
		CVec2D AccelerationDt;
	};

	struct CCollisionInfo {
		CVec2D NormalB;
		CVec2D Point;
		double Depth;
	};

	model::Game game;
	int subtickCount = 10;
	double dTime = 0.1;
	double forwardAccelByType[2] = { 0, 0 };
	double rearAccelByType[2] = { 0, 0 };
	double carLengthwiseFrictionFactorDt = 0;
	double carCrosswiseFrictionFactorDt = 0;
	double carRotationFrictionFactorDt = 0;
	double carMovementAirFrictionFactorDt = 0;
	double carRotationAirFrictionFactorDt = 0;

	CWorldSimulator();

	void updateCar(const CMyMove& move, CMyCar& car, CCarInfo& carInfo, CMyWorld& world) const;
	void moveCar(CCarInfo& carInfo, CMyCar& car) const;
	void moveWasher(CMyWasher& washer) const;
	void moveTire(CMyTire& tire) const;

	void collideTireWithWalls(CMyTire& tire) const;
	void collideTireWithWashers(CMyTire& tire, CMyWorld& world) const;
	void collideTireWithTires(CMyTire& tire, CMyWorld& world) const;
	void collideCarWithWalls(CMyCar& car) const;
	void collideCarWithWashers(int carId, CMyCar& car, CMyWorld& world) const;
	void collideCarWithTires(int carId, CMyCar& car, CMyWorld& world) const;
	void collideCarWithBonuses(CMyCar& car, CMyWorld& world) const;
	void collideCarWithCar(CMyCar& carA, CMyCar& carB, CMyWorld& world) const;

	bool findLineWithCircleCollision(const CVec2D& point1A, const CVec2D& point2A,
		const CVec2D& positionB, double radiusB,
		CCollisionInfo& collisionInfo) const;
	bool findCarWithCarCollision(const CMyCar& carA, const CMyCar& carB,
		CCollisionInfo& collisionInfo) const;
	bool findCarWithCarCollisionPartial(const CMyCar& carA, const CMyCar& carB,
		CCollisionInfo& collisionInfo) const;

	void resolveCollisionStatic(
		const CCollisionInfo& collisionInfo,
		CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRectA, double& collisionDeltaSpeed,
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

	void resolveCollision(
		const CCollisionInfo& collisionInfo,
		CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRectA,
		CVec2D& positionB, CVec2D& speedB, double& angularSpeedB, CRotatedRect& rotatedRectB, double& collisionDeltaSpeed,
		double invertedMassA, double invertedAngularMassA, double invertedMassB, double invertedAngularMassB,
		double momentumTransferFactorAB, double surfaceFrictionFactorAB) const;
	void resolveImpact(
		const CVec3D& vectorAC, const CVec3D& vectorBC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
		CVec2D& speedA, double& angularSpeedA,
		CVec2D& speedB, double& angularSpeedB,
		double invertedMassA, double invertedAngularMassA, double invertedMassB, double invertedAngularMassB,
		double momentumTransferFactorAB) const;
	void resolveSurfaceFriction(
		const CVec3D& vectorAC, const CVec3D& vectorBC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
		CVec2D& speedA, double& angularSpeedA,
		CVec2D& speedB, double& angularSpeedB,
		double invertedMassA, double invertedAngularMassA, double invertedMassB, double invertedAngularMassB,
		double surfaceFrictionFactorAB) const;
	void pushBackBodies(
		const CVec2D& collisionNormalB2D, double depth,
		CVec2D& positionA, CRotatedRect& rotatedRectA,
		CVec2D& positionB, CRotatedRect& rotatedRectB) const;
};
