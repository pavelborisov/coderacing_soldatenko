#include "MyWorld.h"

#include <map>
#include "assert.h"
#include "DrawPlugin.h"
#include "Log.h"

using namespace model;
using namespace std;

CMyBonus CMyWorld::Bonuses[MaxBonuses];
CMyOil CMyWorld::Oils[MaxOils];

CMyWorld::CMyWorld(const World& world, const Car& self)
{
	map<long long, int> carIdMap;
	auto cars = world.getCars();
	// 0 - наша машина
	int nextCarIndex = 0;
	for (const auto& car : cars) {
		if (car.getId() == self.getId()) {
			carIdMap[car.getId()] = nextCarIndex;
			Cars[nextCarIndex++] = CMyCar(car);
			break;
		}
	}
	if (world.getPlayers().size() == 4) {
		// 1, 2, 3 - машины врагов, отсортированы по playerId
		for (int playerId = 1; playerId <= 4; playerId++) {
			if (playerId == self.getPlayerId()) continue;
			for (const auto& car : cars) {
				if (car.getPlayerId() == playerId) {
					carIdMap[car.getId()] = nextCarIndex;
					Cars[nextCarIndex++] = CMyCar(car);
					break;
				}
			}
		}
	} else if (world.getPlayers().size() == 2) {
		// 1 - машина тиммейт
		for (const auto& car : cars) {
			if (car.getId() != self.getId() && car.getPlayerId() == self.getPlayerId()) {
				carIdMap[car.getId()] = nextCarIndex;
				Cars[nextCarIndex++] = CMyCar(car);
				break;
			}
		}
		// 2, 3 - машины врага, отсортированы по типу
		for (int type = 0; type <= 1; type++) {
			for (const auto& car : cars) {
				if (car.getType() == type && car.getPlayerId() != self.getPlayerId()) {
					carIdMap[car.getId()] = nextCarIndex;
					Cars[nextCarIndex++] = CMyCar(car);
				}
			}
		}
	} else {
		assert(false);
	}
	assert(nextCarIndex == MaxCars);

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
				// TODO: –азделить объект на статическую и пременную части.
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
			BonusExist[nextBonusId] = true;
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
			OilTicks[nextOilId] = o.getRemainingLifetime();
			Oils[nextOilId++] = CMyOil(o);
		}
	}
}

void CMyWorld::RemoveInvalidWashers()
{
	int left = 0;
	for (int right = 1; right < MaxWashers; right++) {
		if (Washers[right].IsValid()) {
			while (Washers[left].IsValid() && left < right) {
				left++;
			}
			if (left < right) {
				swap(Washers[left], Washers[right]);
			}
		}
	}
}

void CMyWorld::RemoveInvalidTires()
{
	int left = 0;
	for (int right = 1; right < MaxWashers; right++) {
		if (Tires[right].IsValid()) {
			while (left < right && Tires[left].IsValid()) {
				left++;
			}
			if (left < right) {
				swap(Tires[left], Tires[right]);
			}
		}
	}
}

void CMyWorld::Log() const
{
#ifdef LOGGING
	for (int i = 0; i < MaxCars; i++) {
		CLog::Instance().LogMyCar(Cars[i], "Car");
	}
	for (int i = 0; i < MaxWashers; i++) {
		if (Washers[i].IsValid()) {
			CLog::Instance().Stream() << "Washer: "
				<< Washers[i].Position.X << " " << Washers[i].Position.Y << " "
				<< Washers[i].Speed.X << " " << Washers[i].Speed.Y << endl;
		}
	}
	for (int i = 0; i < MaxTires; i++) {
		if (Tires[i].IsValid()) {
			CLog::Instance().Stream() << "Tire: "
				<< Tires[i].Position.X << " " << Tires[i].Position.Y << " "
				<< Tires[i].Speed.X << " " << Tires[i].Speed.Y << " " << Tires[i].AngularSpeed << endl;
		}
	}
	for (int i = 0; i < MaxOils; i++) {
		if (OilTicks[i] > 0) {
			CLog::Instance().Stream() << "Oil: "
				<< Oils[i].Position.X << " " << Oils[i].Position.Y << " " << OilTicks[i] << endl;
		}
	}
#endif
}

void CMyWorld::LogDifference(const CMyWorld& world) const
{
#ifdef LOGGING
	CLog::Instance().Stream() << "Car[0] check difference" << endl;
	Cars[0].LogDifference(world.Cars[0]);
	CLog::Instance().Stream() << "Car[1] check difference" << endl;
	Cars[1].LogDifference(world.Cars[1]);
	for (int i = 0; i < MaxTires; i++) {
		if (!Tires[i].IsValid()) {
			break;
		}
		CLog::Instance().Stream() << "Tires[i] check difference" << endl;
		Tires[i].LogDifference(world.Tires[i]);
	}
#endif
}

void CMyWorld::Draw(int color) const
{
#ifdef LOGGING
	for (int i = 0; i < MaxCars; i++) {
		for (int j = 0; j < 4; j++) {
			const CVec2D& p1 = Cars[i].RotatedRect.Corners[j];
			const CVec2D& p2 = Cars[i].RotatedRect.Corners[(j + 1) % 4];
			CDrawPlugin::Instance().Line(p1.X, p1.Y, p2.X, p2.Y, color);
		}
	}
	for (int i = 0; i < MaxWashers; i++) {
		if (Washers[i].IsValid()) {
			CDrawPlugin::Instance().Circle(Washers[i].Position.X, Washers[i].Position.Y, CMyWasher::Radius, color);
		}
	}
	for (int i = 0; i < MaxTires; i++) {
		if (Tires[i].IsValid()) {
			CDrawPlugin::Instance().Circle(Tires[i].Position.X, Tires[i].Position.Y, CMyWasher::Radius, color);
		}
	}
#endif
}
