#include "MyStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <assert.h>
#include "GlobalPredictions.h"
#include "Tools.h"
#include "WaypointsDistanceMap.h"

using namespace model;
using namespace std;

template<typename T>
static const void logIfDiffers(const T& a, const T&b, const char* name, CLog& log)
{
	if (abs(a - b) >= 1e-5) {
		log.Log(a - b, (std::string("Prediction error: ") + name).c_str());
	}
}

MyStrategy::MyStrategy() :
	log(CLog::Instance()),
	draw(CDrawPlugin::Instance()),
	currentTick(0),
	nextWaypointIndex(0)
{
}

void MyStrategy::move(const Car& _self, const World& _world, const Game& _game, Move& _resultMove)
{
	self = &_self;
	world = &_world;
	game = &_game;
	resultMove = &_resultMove;

	if (world->getPlayers().size() == 2 && self->getType() == JEEP) return;

	CDrawPluginSwitcher drawSwitcher(draw); 
	currentTick = world->getTick();

	car = CMyCar(*self);
	log.LogTick(currentTick);
	log.LogMyCar(car, "Current            ");

	firstTick();
	updateWaypoints();
	findTileRoute();
	makeMove();
	experiment();
	predict();
	doLog();
	doDraw();

	prevPrediction = prediction;
	prediction.SaveHistory();
}

void MyStrategy::firstTick()
{
	if (currentTick == 0) {
		simulator.Initialize(*game);
	}
}

void MyStrategy::updateWaypoints()
{
	vector<vector<TileType>> tilesXY = world->getTilesXY();
	bool flush = false;
	if (CMyTile::TileTypesXY.size() == 0) {
		flush = true;
		CMyTile::TileTypesXY = tilesXY;
		CMyTile::TileSize = game->getTrackTileSize();
	}
	for (size_t x = 0; x < tilesXY.size(); x++) {
		for (size_t y = 0; y < tilesXY[0].size(); y++) {
			const model::TileType& src = tilesXY[x][y];
			model::TileType& dst = CMyTile::TileTypesXY[x][y];
			if (src != UNKNOWN && src != dst) {
				flush = true;
				dst = src;
			}
		}
	}

	vector<vector<int>> waypoints = world->getWaypoints();
	waypointTiles.clear();
	for (const auto& w : waypoints) {
		waypointTiles.push_back(CMyTile(w[0], w[1]));
	}

	if (flush) {
		CMyTile::FillWalls();
		CWaypointDistanceMap::Instance().Initialize(waypointTiles);
	}

	nextWaypointIndex = self->getNextWaypointIndex();
}

void MyStrategy::findTileRoute()
{
	const int currentX = static_cast<int>(self->getX() / game->getTrackTileSize());
	const int currentY = static_cast<int>(self->getY() / game->getTrackTileSize());
	currentTile = CMyTile(currentX, currentY);

	int dx = 0;
	int dy = 0;
	if (abs(car.Angle) < PI / 4) {
		dx = 1;
	} else if (abs(car.Angle) > 3 * PI / 4) {
		dx = -1;
	} else if (car.Angle > 0) {
		dy = 1;
	} else if (car.Angle < 0) {
		dy = -1;
	}
	tileRoute = tileRouteFinder.FindRoute(waypointTiles, nextWaypointIndex, currentTile, dx, dy);
}

template<class MAP>
void logStats(const MAP& m, const char* name, std::basic_ostream< char, std::char_traits<char> >& stream)
{
	stream << name << ": ";
	for (auto p : m) {
		stream << p.first << "," << p.second << " ";
	}
	stream << endl;
}

void MyStrategy::makeMove()
{
	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(1.0);
		return;
	}

	// Заполняем предсказания
	//predictObjects();

	if (world->getTick() < 1000000) {
		//resultMove->setThrowProjectile(true);
		//resultMove->setBrake(true);
		resultMove->setEnginePower(1.0);
		if (world->getTick() < 227) {
			resultMove->setWheelTurn(-1);
		}
		return;
	}

	CBestMoveFinder bestMoveFinder(car, nextWaypointIndex, *self, *world, *game, waypointTiles, simulator, previousResult);
	CBestMoveFinder::CResult result = bestMoveFinder.Process();
	previousResult = result;
	*resultMove = result.CurrentMove.Convert();
	processShooting();
	processOil();

	// Тупой задний ход
	static int rear = 0;
	double angleToTarget = (tileRoute[1].ToVec() - car.Position).GetAngle();
	double angle = angleToTarget - car.Angle;
	normalizeAngle(angle);
	// Когда симулятор хз что делать.
	if (!result.Success || result.MoveList.back().End < 10) {
		CDrawPlugin::Instance().FillCircle(car.Position.X, car.Position.Y, 50, 0x888888);
		resultMove->setEnginePower(1.0);
		resultMove->setWheelTurn(angle * 32 / PI);
	}
	if (rear == 0) {
		if (self->getDurability() == 0) {
			rear = -game->getCarReactivationTimeTicks() - 50;
		} else if (world->getTick() > 200 && car.Speed.Length() < 1) {
			rear = 120 + static_cast<int>(self->getEnginePower() / game->getCarEnginePowerChangePerTick());
		}
	} else if (rear < 0) {
		rear++;
	} else if (rear > 0) {
		resultMove->setBrake(false);
		CDrawPlugin::Instance().FillCircle(car.Position.X, car.Position.Y, 50, 0x880088);
		if (rear < 30) {
			resultMove->setEnginePower(0);
			resultMove->setBrake(true);
			resultMove->setWheelTurn(0);
		} else {
			resultMove->setEnginePower(-1.0);
			resultMove->setWheelTurn(angle > 0 ? -1 : 1);
		}
		rear--;
		if (rear == 0) rear = -120;
	}

	// Тупое нитро.
	if (result.Success) {
		// Сколько тиков поворачиваем.
		int turnTicks = 0;
		for (const auto& moveWithDuration : result.MoveList) {
			if (moveWithDuration.Move.Turn != 0) {
				turnTicks += moveWithDuration.End - moveWithDuration.Start;
			}
		}
		int totalTicks = result.MoveList.back().End;
		if (self->getNitroChargeCount() > 0 && self->getRemainingNitroCooldownTicks() == 0 && self->getRemainingNitroTicks() == 0
			&& turnTicks <= 15 && totalTicks > 120)
		{
			resultMove->setUseNitro(true);
		}
	}
}

void MyStrategy::predictObjects()
{
	CGlobalPredictions::Clear();

	auto bonuses = world->getBonuses();
	for (const auto& b : bonuses) {
		CGlobalPredictions::Bonuses.push_back({ CVec2D(b.getX(), b.getY()), b.getType(), INT_MAX });
	}

	auto oils = world->getOilSlicks();
	for (const auto& o : oils) {
		CGlobalPredictions::Oils.push_back({ CVec2D(o.getX(), o.getY()), o.getRemainingLifetime() });
	}

	auto projectiles = world->getProjectiles();
	auto cars = world->getCars();
	const int depth = CGlobalPredictions::PredictionDepth;
	model::Move defaultEnemyMove;
	defaultEnemyMove.setEnginePower(1.0);
	for (const auto& c : cars) {
		if (c.getPlayerId() == self->getPlayerId() || c.isFinishedTrack()) continue;
		CGlobalPredictions::EnemyCarsPerTick.push_back(vector<CMyCar>(depth + 1));
		CGlobalPredictions::EnemyCarsPerTick.back()[0] = CMyCar(c);
	}
	for (const auto& p : projectiles) {
		if (p.getType() == WASHER) {
			CMyWasher washer = { CVec2D(p.getX(), p.getY()), CVec2D(p.getSpeedX(), p.getSpeedY()) };
			CGlobalPredictions::WashersPerTick.push_back(vector<CMyWasher>(depth + 1));
			CGlobalPredictions::WashersPerTick.back()[0] = washer;
		} else {
			CMyTire tire = { CVec2D(p.getX(), p.getY()), CVec2D(p.getSpeedX(), p.getSpeedY()), p.getAngularSpeed() };
			CGlobalPredictions::TiresPerTick.push_back(vector<CMyTire>(depth + 1));
			CGlobalPredictions::TiresPerTick.back()[0] = tire;
		}
	}
	for (int tick = 0; tick < depth; tick++) {
		for (size_t i = 0; i < CGlobalPredictions::WashersPerTick.size(); i++) {
			CMyWasher washer = CGlobalPredictions::WashersPerTick[i][tick];
			washer = simulator.Predict(washer, tick);
			CGlobalPredictions::WashersPerTick[i][tick + 1] = washer;
		}
		for (size_t i = 0; i < CGlobalPredictions::TiresPerTick.size(); i++) {
			CMyTire tire = CGlobalPredictions::TiresPerTick[i][tick];
			tire = simulator.Predict(tire, tick);
			CGlobalPredictions::TiresPerTick[i][tick + 1] = tire;
		}
		for (size_t i = 0; i < CGlobalPredictions::EnemyCarsPerTick.size(); i++) {
			CMyCar enemyCar = CGlobalPredictions::EnemyCarsPerTick[i][tick];
			enemyCar = simulator.Predict(enemyCar, defaultEnemyMove, tick);
			CGlobalPredictions::EnemyCarsPerTick[i][tick + 1] = enemyCar;
		}
	}
}

void MyStrategy::processShooting()
{
	if (self->getProjectileCount() == 0) {
		return;
	}

	static const int predictionLength = 40;
	double dmgScore = 0;
	//CDrawPlugin::Instance().SetColor(255, 0, 0);

	vector<double> enemyDurabilities(CGlobalPredictions::EnemyCarsPerTick.size(), 0);
	for (size_t enemyIndex = 0; enemyIndex < CGlobalPredictions::EnemyCarsPerTick.size(); enemyIndex++) {
		enemyDurabilities[enemyIndex] = CGlobalPredictions::EnemyCarsPerTick[enemyIndex][0].Durability;
	}

	int bulletsHit = 0;
	for (int offsetIndex = -1; offsetIndex <= 1; offsetIndex++) {
		double projectileAngle = car.Angle + offsetIndex * game->getSideWasherAngle();
		normalizeAngle(projectileAngle);
		CVec2D projectilePos = car.Position;
		CVec2D projectileSpeed(game->getWasherInitialSpeed(), 0);
		projectileSpeed.Rotate(projectileAngle);
		double projectileRadiusSqr = pow(game->getWasherRadius(), 2);
		//double projectileRadiusSqr = 0; // для повышенной точности
		double projectileDmg = game->getWasherDamage();

		const double carEffectiveRadiusSqr = pow((game->getCarHeight()) / 2, 2);
		
		bool hit = false;
		for (int tick = 1; tick <= predictionLength; tick++) {
			projectilePos += projectileSpeed;
			// TODO: Нормальная проверка коллизий.
			for (size_t enemyIndex = 0; enemyIndex < CGlobalPredictions::EnemyCarsPerTick.size(); enemyIndex++) {
				double& enemyDurability = enemyDurabilities[enemyIndex];
				if (enemyDurability <= 1e-5) {
					continue;
				}
				const CVec2D& enemyPos = CGlobalPredictions::EnemyCarsPerTick[enemyIndex][tick].Position;
				const double distSqr = (enemyPos - projectilePos).LengthSquared();
				if (distSqr < projectileRadiusSqr + carEffectiveRadiusSqr) {
					if (enemyDurability > 1e-5) {
						if (enemyDurability < projectileDmg) {
							dmgScore += enemyDurability * game->getCarDamageScoreFactor();
							dmgScore += game->getCarEliminationScore();
							enemyDurability = 0;
						} else {
							dmgScore += projectileDmg * game->getCarDamageScoreFactor();
							enemyDurability -= projectileDmg;
						}
					}
					//CDrawPlugin::Instance().FillCircle(enemyPos, game->getWasherRadius());
					hit = true;
					bulletsHit++;
					break;
				}
			}
			if (hit) break;
			//CDrawPlugin::Instance().FillCircle(projectilePos, game->getWasherRadius());
		}
	}

	if (dmgScore > 0) {
		log.Stream() << "ApproxDmg: " << dmgScore << "; Bullets hit: " << bulletsHit << endl;
		log.Stream() << enemyDurabilities[0] << " " << enemyDurabilities[1] << " " << enemyDurabilities[2] << endl;
	}

	// Стреляем, если предполагаем, что наберём какой-то минимум очков.
	if (self->getProjectileCount() == 1) {
		if (dmgScore >= 100) {
			resultMove->setThrowProjectile(true);
		}
	} else if (self->getProjectileCount() > 1) {
		if (dmgScore >= 40) {
			resultMove->setThrowProjectile(true);
		}
	}
}

void MyStrategy::processOil()
{
	if (self->getOilCanisterCount() == 0) {
		return;
	}

	static const int predictionLength = 50;
	static const double minEnemySpeed = 5;
	static const double minEnemyAngularSpeed = PI / 90;
	static const double minEnemyDrift = PI / 20;
	double speedRelAngle = car.Speed.GetAngle() - car.Angle;
	normalizeAngle(speedRelAngle);
	if (abs(speedRelAngle) > PI / 2) {
		// Мы сбросим лужу себе под колёса.
		return;
	}

	const double oilRadius = game->getOilSlickRadius();
	CVec2D oilOffset(game->getOilSlickInitialRange() + game->getCarWidth() / 2 + oilRadius, 0);
	oilOffset.Rotate(car.Angle);
	const CVec2D oilPos = car.Position - oilOffset;
	//CDrawPlugin::Instance().SetColor(196, 196, 196);
	//CDrawPlugin::Instance().FillCircle(oilPos, oilRadius);

	// Выбрасываем лужу только, когда мы уверены, что в неё попадёт какой-либо враг сзади.
	// Попадание в мазут считается если центр машины попал внутрь окружности мазута.
	const double oilRadiusSqr = pow(oilRadius, 2);
	for (size_t enemyIndex = 0; enemyIndex < CGlobalPredictions::EnemyCarsPerTick.size(); enemyIndex++) {
		for (int tick = 1; tick <= predictionLength; tick++) {
			const CMyCar& enemy = CGlobalPredictions::EnemyCarsPerTick[enemyIndex][tick];
			// Проверка, что автомобиль попал.
			if ((enemy.Position - oilPos).LengthSquared() < oilRadiusSqr) {
				double enemySpeedRelAngle = enemy.Speed.GetAngle() - enemy.Angle;
				normalizeAngle(enemySpeedRelAngle);
				double enemyDrift = abs(enemySpeedRelAngle);
				enemyDrift = enemyDrift > PI / 2 ? PI - enemyDrift : enemyDrift;
				if (enemy.Speed.Length() > minEnemySpeed && 
					(enemyDrift > minEnemyDrift || abs(enemy.AngularSpeed > minEnemyAngularSpeed)))
				{
					resultMove->setSpillOil(true);
					break;
				}
			}
		}
	}
}
void MyStrategy::experiment()
{
	for (const auto& p : CGlobalPredictions::WashersPerTick) {
		for (int i = 0; i < 3; i++) {
			CLog::Instance().Stream() << "Washer Tick " << i << ": "
				<< p[i].Position.X << "," << p[i].Position.Y << " "
				<< p[i].Speed.X << "," << p[i].Speed.Y << endl;
		}
	}
	for (const auto& t : CGlobalPredictions::TiresPerTick) {
		for (int i = 0; i < 3; i++) {
			CLog::Instance().Stream() << "Tire Tick " << i << ": "
				<< t[i].Position.X << "," << t[i].Position.Y << " "
				<< t[i].Speed.X << "," << t[i].Speed.Y << " "
				<< t[i].AngularSpeed << endl;
		}
	}
}

void MyStrategy::predict()
{
	prediction = simulator.Predict(car, *resultMove, 0);
}

void MyStrategy::doLog()
{
	log.LogMyCar(prediction, "Prediction         ");

	if (currentTick > 180) {
		logIfDiffers(car.Angle, prevPrediction.Angle, "Angle", log);
		logIfDiffers(car.AngularSpeed, prevPrediction.AngularSpeed, "AngularSpeed", log);
		logIfDiffers(car.EnginePower, prevPrediction.EnginePower, "EnginePower", log);
		logIfDiffers(car.WheelTurn, prevPrediction.WheelTurn, "WheelTurn", log);
		logIfDiffers(car.Durability, prevPrediction.Durability, "Durability", log);
		logIfDiffers(car.NitroCooldown, prevPrediction.NitroCooldown, "NitroCooldown", log);
		logIfDiffers(car.NitroCount, prevPrediction.NitroCount, "NitroCount", log);
		logIfDiffers(car.NitroTicks, prevPrediction.NitroTicks, "NitroTicks", log);
		logIfDiffers(car.OiledTicks, prevPrediction.OiledTicks, "OiledTicks", log);
		logIfDiffers(car.Position.X, prevPrediction.Position.X, "Position.X", log);
		logIfDiffers(car.Position.Y, prevPrediction.Position.Y, "Position.Y", log);
		logIfDiffers(car.Speed.X, prevPrediction.Speed.X, "Speed.X", log);
		logIfDiffers(car.Speed.Y, prevPrediction.Speed.Y, "Speed.Y", log);
		logIfDiffers(car.Type, prevPrediction.Type, "Type", log);
	}
}

void MyStrategy::doDraw()
{
	CVec2D nextWaypoint = waypointTiles[nextWaypointIndex].ToVec();
	draw.FillCircle(nextWaypoint.X, nextWaypoint.Y, 50, 0xFF0000);

	for (const auto& c : prediction.RotatedRect.Corners) {
		draw.FillCircle(c.X, c.Y, 1, 0x005500);
	}

	//for (size_t i = 1; i < min(10U, tileRoute.size()); i++) {
	//	CVec2D from = tileRoute[i - 1].ToVec();
	//	CVec2D to = tileRoute[i].ToVec();
	//	draw.Line(from.X, from.Y, to.X, to.Y, 0x00FF00);
	//}
}
