#include "MyWorld.h"

#include <algorithm>
#include <vector>
#include "assert.h"
#include "DrawPlugin.h"
#include "Log.h"

using namespace model;
using namespace std;

CMyBonus CMyWorld::Bonuses[MaxBonuses];
CMyOil CMyWorld::Oils[MaxOils];
int CMyWorld::PlayersCount = 0;
map<long long, int> CMyWorld::PlayerIdMap;
map<long long, int> CMyWorld::CarIdMap;

CMyWorld::CMyWorld(const World& world, const Car& self)
{
	PlayerIdMap.clear();
	auto players = world.getPlayers();
	PlayersCount = players.size();
	int nextPlayerIndex = 0;
	// 0 - наш игрок
	for (const auto& p : players) {
		if (p.getId() == self.getPlayerId()) {
			PlayerIdMap[p.getId()] = nextPlayerIndex;
			Players[nextPlayerIndex++] = CMyPlayer(p);
		}
	}
	// 1, 2, 3 - чужие игроки
	for (int playerId = 1; playerId <= 4; playerId++) {
		if (playerId == self.getPlayerId()) continue;
		for (const auto& p : players) {
			if (p.getId() == playerId) {
				PlayerIdMap[p.getId()] = nextPlayerIndex;
				Players[nextPlayerIndex++] = CMyPlayer(p);
				break;
			}
		}
	}

	CarIdMap.clear();
	for (int i = 0; i < MaxCars; i++) {
		Cars[i].Invalidate();
	}
	auto cars = world.getCars();
	// 0 - наша машина
	int nextCarIndex = 0;
	for (const auto& car : cars) {
		if (car.getId() == self.getId()) {
			CarIdMap[car.getId()] = nextCarIndex;
			Cars[nextCarIndex++] = CMyCar(car, PlayerIdMap[car.getPlayerId()]);
			break;
		}
	}
	if (world.getPlayers().size() == 4) {
		// 1, 2, 3 - машины врагов, отсортированы по playerId
		for (int playerId = 1; playerId <= 4; playerId++) {
			if (playerId == self.getPlayerId()) continue;
			for (const auto& car : cars) {
				if (car.getPlayerId() == playerId) {
					CarIdMap[car.getId()] = nextCarIndex;
					Cars[nextCarIndex++] = CMyCar(car, PlayerIdMap[car.getPlayerId()]);
					break;
				}
			}
		}
	} else if (world.getPlayers().size() == 2) {
		// 1 - машина тиммейт
		for (const auto& car : cars) {
			if (car.getId() != self.getId() && car.getPlayerId() == self.getPlayerId()) {
				CarIdMap[car.getId()] = nextCarIndex;
				Cars[1] = CMyCar(car, PlayerIdMap[car.getPlayerId()]);
				break;
			}
		}
		if (!Cars[1].IsValid()) {
			Cars[1].PlayerId = Cars[0].PlayerId;
			Cars[1].Type = 1 - Cars[0].Type;
		}
		nextCarIndex++;

		// 2, 3 - машины врага, отсортированы по типу
		for (const auto& car : cars) {
			if (car.getType() == BUGGY && car.getPlayerId() != self.getPlayerId()) {
				CarIdMap[car.getId()] = nextCarIndex;
				Cars[2] = CMyCar(car, PlayerIdMap[car.getPlayerId()]);
			}
		}
		if (!Cars[2].IsValid()) {
			Cars[2].PlayerId = 1;
			Cars[2].Type = 0;
		}
		nextCarIndex++;

		for (const auto& car : cars) {
			if (car.getType() == JEEP && car.getPlayerId() != self.getPlayerId()) {
				CarIdMap[car.getId()] = nextCarIndex;
				Cars[3] = CMyCar(car, PlayerIdMap[car.getPlayerId()]);
			}
		}
		if (!Cars[3].IsValid()) {
			Cars[3].PlayerId = 1;
			Cars[3].Type = 0;
		}
		nextCarIndex++;
	} else {
		assert(false);
	}
	assert(nextCarIndex == MaxCars);

	auto projectiles = world.getProjectiles();
	sort(projectiles.begin(), projectiles.end(), [](const auto& a, const auto& b) {return a.getId() < b.getId(); });
	int nextWasherId = 0;
	int nextTireId = 0;
	for (const auto& p : projectiles) {
		const long long carId = p.getCarId();
		if (p.getType() == WASHER) {
			if (nextWasherId < MaxWashers) {
				if (CarIdMap.find(carId) == CarIdMap.end()) {
					Washers[nextWasherId++] = CMyWasher(p, 2);
				} else {
					Washers[nextWasherId++] = CMyWasher(p, CarIdMap[p.getCarId()]);
				}
			} else {
				CLog::Instance().Stream() << "Warning! Too many washers" << endl;
				break;
			}
		} else if (p.getType() == TIRE) {
			if (nextTireId < MaxTires) {
				if (CarIdMap.find(carId) == CarIdMap.end()) {
					Tires[nextTireId++] = CMyTire(p, 3);
				} else {
					Tires[nextTireId++] = CMyTire(p, CarIdMap[p.getCarId()]);
				}
			} else {
				CLog::Instance().Stream() << "Warning! Too many tires" << endl;
				break;
			}
		} else {
			assert(false);
		}
	}

	// Бонусы отсортируем по близости к машине.
	auto bonuses = world.getBonuses();
	vector<pair<double, int>> bonusDistanceIndex;
	for (size_t i = 0; i < bonuses.size(); i++) {
		const double distance = (CVec2D(bonuses[i].getX(), bonuses[i].getY()) - Cars[0].Position).Length();
		bonusDistanceIndex.push_back(make_pair(distance, i));
	}
	sort(bonusDistanceIndex.begin(), bonusDistanceIndex.end());
	int nextBonusId = 0;
	for (int i = 0; i < MaxBonuses; i++) {
		BonusExist[i] = false;
	}
	for (const auto& bdi : bonusDistanceIndex) {
		if (nextBonusId < MaxBonuses) {
			BonusExist[nextBonusId] = true;
			Bonuses[nextBonusId++] = CMyBonus(bonuses[bdi.second]);
		} else {
			CLog::Instance().Stream() << "Warning! Too many bonuses. Stopping at "
				<< bonusDistanceIndex[nextBonusId - 1].first << " distance" << endl;
			break;
		}
	}

	auto oils = world.getOilSlicks();
	sort(oils.begin(), oils.end(), [](const auto& a, const auto& b) { return a.getId() < b.getId(); });
	int nextOilId = 0;
	for (int i = 0; i < MaxOils; i++) {
		OilTicks[i] = 0;
	}
	for (const auto& o : oils) {
		if (nextOilId < MaxOils) {
			OilTicks[nextOilId] = o.getRemainingLifetime();
			Oils[nextOilId++] = CMyOil(o);
		} else {
			CLog::Instance().Stream() << "Warning! Too many oils" << endl;
			break;
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
	for (int right = 1; right < MaxTires; right++) {
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
	CLog::Instance().Stream() << "Players[0] check difference" << endl;
	Players[0].LogDifference(world.Players[0]);
	//CLog::Instance().Stream() << "Players[1] check difference" << endl;
	//Players[1].LogDifference(world.Players[1]);
	CLog::Instance().Stream() << "Cars[0] check difference" << endl;
	Cars[0].LogDifference(world.Cars[0]);
	//CLog::Instance().Stream() << "Car[1] check difference" << endl;
	//Cars[1].LogDifference(world.Cars[1]);
	for (int i = 0; i < MaxTires; i++) {
		if (!Tires[i].IsValid()) {
			break;
		}
		CLog::Instance().Stream() << "Tires[i] check difference" << endl;
		Tires[i].LogDifference(world.Tires[i]);
	}
#else
	world;
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
#else
	color;
#endif
}
