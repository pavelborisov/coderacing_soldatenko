#pragma once

#include "model\car.h"
#include "model\world.h"
#include "MyCar.h"
#include "MyObjects.h"

struct CMyWorld {
	static const int MaxCars = 4;
	static const int MaxWashers = 3 * 4 * 2;
	static const int MaxTires = 4 * 2;
	static const int MaxBonuses = 20; // TODO: сколько?
	static const int MaxOils = 10;

	CMyCar Cars[MaxCars]; // в нуле - наша машина, в единице - возможно союзник, остальные - враги.
	CMyWasher Washers[MaxWashers];
	CMyTire Tires[MaxTires];
	static CMyBonus Bonuses[MaxBonuses];
	bool BonusExist[MaxBonuses];
	static CMyOil Oils[MaxOils];
	int OilTicks[MaxOils];

	CMyWorld(const model::World& world, const model::Car& self);
};
