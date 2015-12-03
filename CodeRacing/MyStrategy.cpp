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
	predictObjects();
	predictEnemyPositions();

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

	//auto projectiles = world->getProjectiles();
	//CGlobalPredictions::WashersPerTick;
	//for (auto& p : projectiles) {
	//	for (int tick = 0; tick < CGlobalPredictions::PredictionDepth; tick++) {
	//		if (p.getType() == WASHER) {
	//			if (tick == 0) {
	//				CGlobalPredictions::WashersPerTick[tick].push_back({ CVec2D(p.getX(), p.getY()), CVec2D(p.getSpeedX(), p.getSpeedY()) });
	//			}
	//			
	//		} else {

	//		}
	//	}
	//}
}

void MyStrategy::predictEnemyPositions()
{
	static const int predictionLength = 50;
	enemyPredictions.clear();
	enemyCars.clear();
	//CDrawPlugin::Instance().SetColor(128, 255, 0);
	for (const auto& otherCar : world->getCars()) {
		if (otherCar.getPlayerId() != self->getPlayerId()) {
			vector<CMyCar> predictions(predictionLength + 1);
			predictions[0] = CMyCar(otherCar);
			model::Move simpleMove;
			simpleMove.setEnginePower(otherCar.getEnginePower());
			for (int tick = 1; tick <= predictionLength; tick++) {
				predictions[tick] = simulator.Predict(predictions[tick - 1], *world, simpleMove, tick); // TODO: check off by one error (tick)
				//CDrawPlugin::Instance().FillCircle(predictions[tick].Position, 5);
			}
			enemyPredictions.emplace_back(predictions);
			enemyCars.push_back(otherCar);
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

	vector<double> enemyDurabilities(enemyPredictions.size(), 0);
	for (size_t enemyIndex = 0; enemyIndex < enemyPredictions.size(); enemyIndex++) {
		enemyDurabilities[enemyIndex] = enemyCars[enemyIndex].getDurability();
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
			for (size_t enemyIndex = 0; enemyIndex < enemyPredictions.size(); enemyIndex++) {
				double& enemyDurability = enemyDurabilities[enemyIndex];
				if (enemyCars[enemyIndex].isFinishedTrack() || enemyDurability <= 1e-5) {
					continue;
				}
				const CVec2D& enemyPos = enemyPredictions[enemyIndex][tick].Position;
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
	for (size_t enemyIndex = 0; enemyIndex < enemyPredictions.size(); enemyIndex++) {
		for (int tick = 1; tick <= predictionLength; tick++) {
			const CMyCar& enemy = enemyPredictions[enemyIndex][tick];
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
	//double dist = CWaypointDistanceMap::Instance().Query(car.Position.X, car.Position.Y, car.Angle, nextWaypointIndex);
	//log.Stream() << dist;
	//for (int tileX = 0; tileX < CMyTile::SizeX(); tileX++) {
	//	const int startX = tileX * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//	const int endX = (tileX + 1) * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//	for (int tileY = 0; tileY < CMyTile::SizeY(); tileY++) {
	//		const int startY = tileY * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//		const int endY = (tileY + 1) * CWaypointDistanceMap::tileSize / CWaypointDistanceMap::step;
	//		for (int x = startX; x < endX; x++) {
	//			for (int y = startY; y < endY; y++) {
	//				const double worldX = (x + 0.5) * CWaypointDistanceMap::step;
	//				const double worldY = (y + 0.5) * CWaypointDistanceMap::step;
	//				//dist = CWaypointDistanceMap::Instance().QueryBestDirection(worldX, worldY, nextWaypointIndex);
	//				dist = CWaypointDistanceMap::Instance().Query(worldX, worldY, car.Angle, nextWaypointIndex);
	//				//dist;
	//				CDrawPlugin::Instance().Text(worldX, worldY, to_string((int)dist).c_str(), 0x000000);
	//			}
	//		}
	//	}
	//}
}

void MyStrategy::predict()
{
	prediction = simulator.Predict(car, *world, *resultMove, 1);
}

void MyStrategy::doLog()
{
	log.LogTick(currentTick);
	log.LogMyCar(car,        "Current            ");
	log.LogMyCar(prediction, "Prediction         ");

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

void MyStrategy::doDraw()
{
	CVec2D nextWaypoint = waypointTiles[nextWaypointIndex].ToVec();
	draw.FillCircle(nextWaypoint.X, nextWaypoint.Y, 50, 0xFF0000);

	//for (size_t i = 1; i < min(10U, tileRoute.size()); i++) {
	//	CVec2D from = tileRoute[i - 1].ToVec();
	//	CVec2D to = tileRoute[i].ToVec();
	//	draw.Line(from.X, from.Y, to.X, to.Y, 0x00FF00);
	//}
}
