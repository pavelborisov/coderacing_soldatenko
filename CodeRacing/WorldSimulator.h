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

};
