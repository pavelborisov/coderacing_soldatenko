#pragma once

#include <map>
#include <vector>
#include "model\car.h"
#include "model\world.h"
#include "MyCar.h"
#include "MyObjects.h"
#include "MyTile.h"
#include "Tools.h"

struct CMyWorld {
	static const int MaxPlayers = 4;
	static const int MaxCars = 4;
	static const int MaxWashers = 3 * 4 * 2;
	static const int MaxTires = 4 * 2;
	static const int MaxBonuses = 20;
	static const int MaxOils = 10;

	static std::vector<CMyTile> WaypointTiles;
	static TDirection StartDirection;

	static std::map<long long, int> PlayerIdMap;
	static int PlayersCount;
	CMyPlayer Players[MaxPlayers]; // в нуле - наш игрок, остальные - чужие.

	static std::map<long long, int> CarIdMap;
	CMyCar Cars[MaxCars]; // в нуле - наша машина, в единице - возможно союзник, остальные - враги.

	CMyWasher Washers[MaxWashers];
	CMyTire Tires[MaxTires];
	static CMyBonus Bonuses[MaxBonuses];
	bool BonusExist[MaxBonuses];
	static CMyOil Oils[MaxOils];
	int OilTicks[MaxOils];

	CMyWorld() {}
	CMyWorld(const model::World& world, const model::Car& self);

	void RemoveInvalidWashers();
	void RemoveInvalidTires();

	void Log() const;
	void LogDifference(const CMyWorld& world) const;
	void Draw(int color) const;
};
