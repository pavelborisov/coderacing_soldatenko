#include "MyStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <assert.h>
#include "BestMoveFinder.h"
#include "Tools.h"

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
	draw.BeginDraw();
	draw.EndDraw();
}

void MyStrategy::move(const Car& _self, const World& _world, const Game& _game, Move& _resultMove)
{
	self = &_self;
	world = &_world;
	game = &_game;
	resultMove = &_resultMove;

	CDrawPluginSwitcher drawSwitcher(draw); 
	currentTick = world->getTick();

	car = CMyCar(*self);
	firstTick();
	updateWaypoints();
	findTileRoute();
	makeMove();
	predict();
	doLog();
	doDraw();

	prevPrediction = prediction;
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
	if (CMyTile::TileTypesXY.size() == 0) {
		CMyTile::TileTypesXY = tilesXY;
		CMyTile::TileSize = game->getTrackTileSize();
	}
	for (size_t x = 0; x < tilesXY.size(); x++) {
		for (size_t y = 0; y < tilesXY.size(); y++) {
			if (tilesXY[x][y] != UNKNOWN) {
				CMyTile::TileTypesXY[x][y] = tilesXY[x][y];
			}
		}
	}

	vector<vector<int>> waypoints = world->getWaypoints();
	waypointTiles.clear();
	for (const auto& w : waypoints) {
		waypointTiles.push_back(CMyTile(w[0], w[1]));
	}

	nextWaypointIndex = self->getNextWaypointIndex();
}

void MyStrategy::findTileRoute()
{
	const int currentX = static_cast<int>(self->getX() / game->getTrackTileSize());
	const int currentY = static_cast<int>(self->getY() / game->getTrackTileSize());
	currentTile = CMyTile(currentX, currentY);

	// TODO: Сделать поиск маршрута в зависимости от направления скорости машины в начальной точке.
	tileRoute = tileRouteFinder.FindRoute(waypointTiles, nextWaypointIndex, currentTile);
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

	predictEnemyPositions();

	CBestMoveFinder bestMoveFinder(car, *world, *game, tileRoute, simulator);
	CBestMoveFinder::CResult result = bestMoveFinder.Process();
	*resultMove = result.CurrentMove.Convert();
	processShooting();
	processOil();

	// Тупой задний ход
	// TODO: Запомнить, куда мы поворачивали колеса в последний раз и задним ходом делать наборот.
	static int rear = 0;
	double angleToTarget = (tileRoute[1].ToVec() - car.Position).GetAngle();
	double angle = angleToTarget - car.Angle;
	normalizeAngle(angle);
	// Когда симулятор хз что делать.
	if (!result.Success || result.MoveList.back().End < 5) {
		CDrawPlugin::Instance().SetColor(128, 128, 128);
		CDrawPlugin::Instance().FillCircle(car.Position, 50);
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
		CDrawPlugin::Instance().SetColor(128, 0, 128);
		CDrawPlugin::Instance().FillCircle(car.Position, 50);
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

	// TODO: Проверка на прямые участки.
	// Тупое нитро.
	// Сколько тиков поворачиваем.
	if (result.Success) {
		int turnTicks = 0;
		for (const auto& moveWithDuration : result.MoveList) {
			if (moveWithDuration.Move.Turn != 0) {
				turnTicks += moveWithDuration.End - moveWithDuration.Start;
			}
		}
		int totalTicks = result.MoveList.back().End;
		if (self->getNitroChargeCount() > 0 && self->getRemainingNitroCooldownTicks() == 0 && self->getRemainingNitroTicks() == 0
			&& turnTicks < 20 && totalTicks > 120)
		{
			resultMove->setUseNitro(true);
		}
	}
}

void MyStrategy::predictEnemyPositions()
{
	static const int predictionLength = 50;
	enemyPredictions.clear();
	//CDrawPlugin::Instance().SetColor(128, 255, 0);
	for (const auto& otherCar : world->getCars()) {
		if (otherCar.getPlayerId() != self->getPlayerId()) {
			vector<CMyCar> predictions(predictionLength + 1);
			predictions[0] = CMyCar(otherCar);
			model::Move simpleMove;
			simpleMove.setEnginePower(otherCar.getEnginePower());
			simpleMove.setWheelTurn(otherCar.getWheelTurn());
			for (int tick = 1; tick <= predictionLength; tick++) {
				predictions[tick] = simulator.Predict(predictions[tick - 1], *world, simpleMove);
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

	static const int predictionLength = 50;
	double dmgScore = 0;
	//CDrawPlugin::Instance().SetColor(255, 0, 0);
	for (int offsetIndex = -1; offsetIndex <= 1; offsetIndex++) {
		double projectileAngle = car.Angle + offsetIndex * game->getSideWasherAngle();
		normalizeAngle(projectileAngle);
		CVec2D projectilePos = car.Position;
		CVec2D projectileSpeed(game->getWasherInitialSpeed(), 0);
		projectileSpeed.Rotate(projectileAngle);
		double projectileRadiusSqr = pow(game->getWasherRadius(), 2);
		double projectileDmg = game->getWasherDamage();

		const double carEffectiveRadiusSqr = pow((game->getCarHeight()) / 2, 2);
		
		for (int tick = 1; tick <= predictionLength; tick++) {
			projectilePos += projectileSpeed;
			// TODO: Нормальная проверка коллизий.
			for (size_t enemyIndex = 0; enemyIndex < enemyPredictions.size(); enemyIndex++ ) {
				const model::Car& enemyCar = enemyCars[enemyIndex];
				if (enemyCar.isFinishedTrack() || enemyCar.getDurability() == 0) {
					continue;
				}
				const CVec2D& enemyPos = enemyPredictions[enemyIndex][tick].Position;
				const double distSqr = (enemyPos - projectilePos).LengthSquared();
				if (distSqr < projectileRadiusSqr + carEffectiveRadiusSqr) {
					dmgScore += min(enemyCar.getDurability(), projectileDmg) * game->getCarDamageScoreFactor();
					if (enemyCar.getDurability() < projectileDmg) {
						dmgScore += game->getCarEliminationScore();
					}
					//CDrawPlugin::Instance().FillCircle(enemyPos, game->getWasherRadius());
					break;
				}
			}
			//CDrawPlugin::Instance().FillCircle(projectilePos, game->getWasherRadius());
		}
	}

	// Стреляем, если предполагаем, что наберём какой-то минимум очков.
	if (dmgScore > 40) {
		resultMove->setThrowProjectile(true);
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

void MyStrategy::predict()
{
	prediction = simulator.Predict(car, *world, *resultMove);
}

void MyStrategy::doLog()
{
	log.LogTick(currentTick);
	log.LogMyCar(car,        "Current            ");
	log.LogMyCar(prediction, "Prediction         ");

	logIfDiffers(car.Angle, prevPrediction.Angle, "Angle", log);
	logIfDiffers(car.AngularSpeed, prevPrediction.AngularSpeed, "AngularSpeed", log);
	logIfDiffers(car.EnginePower, prevPrediction.EnginePower, "EnginePower", log);
	logIfDiffers(car.NitroCooldown, prevPrediction.NitroCooldown, "NitroCooldown", log);
	logIfDiffers(car.NitroCount, prevPrediction.NitroCount, "NitroCount", log);
	logIfDiffers(car.NitroTicks, prevPrediction.NitroTicks, "NitroTicks", log);
	logIfDiffers(car.Position.X, prevPrediction.Position.X, "Position.X", log);
	logIfDiffers(car.Position.Y, prevPrediction.Position.Y, "Position.Y", log);
	logIfDiffers(car.Speed.X, prevPrediction.Speed.X, "Speed.X", log);
	logIfDiffers(car.Speed.Y, prevPrediction.Speed.Y, "Speed.Y", log);
	logIfDiffers(car.Type, prevPrediction.Type, "Type", log);
}

void MyStrategy::doDraw()
{
	draw.SetColor(0, 0, 0);
	draw.FillCircle(car.Position, 10);
	draw.SetColor(255, 128, 0);
	draw.FillCircle(prediction.Position, 5);

	draw.SetColor(255, 0, 0);
	CVec2D nextWaypoint = waypointTiles[nextWaypointIndex].ToVec();
	draw.FillCircle(nextWaypoint, 50);

	draw.SetColor(0, 255, 0);
	for (size_t i = 1; i < min(10U, tileRoute.size()); i++) {
		CVec2D from = tileRoute[i - 1].ToVec();
		CVec2D to = tileRoute[i].ToVec();
		draw.DrawLine(from, to);
	}
}
