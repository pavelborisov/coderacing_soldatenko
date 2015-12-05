#include "MyWorld.h"

#include "assert.h"
#include "Log.h"
#include <map>

using namespace model;
using namespace std;

CMyBonus CMyWorld::Bonuses[MaxBonuses];
CMyOil CMyWorld::Oils[MaxOils];

CMyWorld::CMyWorld(const World& world, const Car& self)
{
	map<long long, int> carIdMap;
	auto cars = world.getCars();
	int nextCarIndex = 0;
	for (const auto& car : cars) {
		if (car.getId() == self.getId()) {
			carIdMap[car.getId()] = nextCarIndex;
			Cars[nextCarIndex++] = CMyCar(car);
			break;
		}
	}
	for (const auto& car : cars) {
		if (car.getId() != self.getId() && car.getPlayerId() == self.getPlayerId()) {
			carIdMap[car.getId()] = nextCarIndex;
			Cars[nextCarIndex++] = CMyCar(car);
			break;
		}
	}
	for (const auto& car : cars) {
		if (car.getPlayerId() != self.getPlayerId()) {
			assert(nextCarIndex < MaxCars);
			carIdMap[car.getId()] = nextCarIndex;
			Cars[nextCarIndex++] = CMyCar(car);
		}
	}

	auto projectiles = world.getProjectiles();
	int nextWasherId = 0;
	int nextTireId = 0;
	for (const auto& p : projectiles) {
		if (p.getType() == WASHER) {
			if (nextWasherId < MaxWashers) {
				Washers[nextWasherId++] = CMyWasher(p, carIdMap[p.getCarId()]);
			} else {
				CLog::Instance().Stream() << "Too many washers" << endl;
			}
		} else if (p.getType() == TIRE) {
			if (nextTireId < MaxTires) {
				// TODO: Разделить объект на статическую и пременную части.
				Tires[nextTireId++] = CMyTire(p, carIdMap[p.getCarId()]);
			} else {
				CLog::Instance().Stream() << "Too many tires" << endl;
			}
		} else {
			assert(false);
		}
	}

	auto bonuses = world.getBonuses();
	int nextBonusId = 0;
	for (int i = 0; i < MaxBonuses; i++) {
		BonusExist[i] = false;
	}
	for (const auto& b : bonuses) {
		if (nextBonusId < MaxBonuses) {
			Bonuses[nextBonusId++] = CMyBonus(b);
		} else {
			CLog::Instance().Stream() << "Too many bonuses" << endl;
		}
	}

	auto oils = world.getOilSlicks();
	int nextOilId = 0;
	for (int i = 0; i < MaxOils; i++) {
		OilTicks[i] = 0;
	}
	for (const auto& o : oils) {
		if (nextOilId < MaxOils) {
			Oils[nextOilId++] = CMyOil(o);
		}
	}
}
