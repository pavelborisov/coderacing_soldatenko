#pragma once

#include "model\Game.h"
#include "MyMove.h"
#include "MyWorld.h"

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
	void collideCarWithWalls(CMyCar& car) const;
	void collideCarWithWashers(CMyCar& car, CMyWorld& world) const;
	void collideCarWithTires(CMyCar& car, CMyWorld& world) const;
	void collideCarWithBonuses(CMyCar& car, CMyWorld& world) const;
	void collideCarWithCar(CMyCar& carA, CMyCar& carB) const;

	bool findCarWithLeftWallCollision(const CMyCar& car, double xWall, CCollisionInfo& collisionInfo) const;
};
