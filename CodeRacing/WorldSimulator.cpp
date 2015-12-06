#include "WorldSimulator.h"

#ifdef LOGGING
#undef NDEBUG
#endif

#include <algorithm>
#include "assert.h"
#include "Arc2D.h"
#include "Line2D.h"
#include "MyTile.h"
#include "Tools.h"

using namespace std;

static const double epsilon = 1e-7;
static const double wallOffsetMin = CMyTile::WallRadius;
static const double wallOffsetMin2 = CMyTile::WallRadius * 2;
static const double wallOffsetMax = CMyTile::TileSize - CMyTile::WallRadius;
static const double wallOffsetMax2 = CMyTile::TileSize - 2 * CMyTile::WallRadius;
static const double minTireSpeed = 60 * 0.25;

static const double limit(double val, double lim)
{
	return max(-lim, min(lim, val));
}

CWorldSimulator::CWorldSimulator()
{
}

void CWorldSimulator::SetGame(const model::Game& _game)
{
	game = _game;
}

void CWorldSimulator::SetPrecision(int _subtickCount)
{
	subtickCount = _subtickCount;
	dTime = 1.0 / subtickCount;

	forwardAccelByType[0] = game.getBuggyEngineForwardPower() / game.getBuggyMass();
	forwardAccelByType[1] = game.getJeepEngineForwardPower() / game.getJeepMass();
	rearAccelByType[0] = game.getBuggyEngineRearPower() / game.getBuggyMass();
	rearAccelByType[1] = game.getJeepEngineRearPower() / game.getJeepMass();

	carLengthwiseFrictionFactorDt = game.getCarLengthwiseMovementFrictionFactor() * dTime;
	carCrosswiseFrictionFactorDt = game.getCarCrosswiseMovementFrictionFactor() * dTime;
	carRotationFrictionFactorDt = game.getCarRotationFrictionFactor() * dTime;
	carMovementAirFrictionFactorDt = pow(1 - game.getCarMovementAirFrictionFactor(), dTime);
	carRotationAirFrictionFactorDt = pow(1 - game.getCarRotationAirFrictionFactor(), dTime);
}

CMyWorld CWorldSimulator::Simulate(const CMyWorld& startWorld, const CMyMove moves[CMyWorld::MaxCars]) const
{
	// TODO: NextWaypointIndex
	CMyWorld world = startWorld;
	CCarInfo carInfos[CMyWorld::MaxCars];
	for (int i = 0; i < CMyWorld::MaxCars; i++) {
		if (world.Cars[i].IsFinished) {
			continue;
		}
		updateCar(moves[i], world.Cars[i], carInfos[i], world);
	}

	for (int subtick = 0; subtick < subtickCount; subtick++) {
		// Сначала всё двигаем.
		for (int i = 0; i < CMyWorld::MaxCars; i++) {
			if (world.Cars[i].IsFinished) {
				continue;
			}
			moveCar(carInfos[i], world.Cars[i]);
		}
		for (int i = 0; i < CMyWorld::MaxWashers; i++) {
			if (!world.Washers[i].IsValid()) {
				break;
			}
			moveWasher(world.Washers[i]);
		}
		for (int i = 0; i < CMyWorld::MaxTires; i++) {
			if (!world.Tires[i].IsValid()) {
				break;
			}
			moveTire(world.Tires[i]);
		}

		// Теперь разбираемся с коллизиями. Какой правильный порядок - хз.
		bool shouldRemoveInvalidTires = false;
		for (int i = 0; i < CMyWorld::MaxTires; i++) {
			if (!world.Tires[i].IsValid()) {
				break;
			}
			collideTireWithWalls(world.Tires[i]);
			collideTireWithWashers(world.Tires[i], world);
			if (world.Tires[i].Speed.Length() < minTireSpeed) {
				world.Tires[i].Invalidate();
				shouldRemoveInvalidTires = true;
			}
		}
		if (shouldRemoveInvalidTires) {
			world.RemoveInvalidTires();
		}

		for (int i = 0; i < CMyWorld::MaxCars; i++) {
			if (world.Cars[i].IsFinished) {
				continue;
			}
			collideCarWithWalls(world.Cars[i]);
			collideCarWithWashers(i, world.Cars[i], world);
			collideCarWithTires(i, world.Cars[i], world);
			collideCarWithBonuses(world.Cars[i], world);
			for (int j = i + 1; j < CMyWorld::MaxCars; j++) {
				if (world.Cars[j].IsFinished) {
					continue;
				}
				collideCarWithCar(world.Cars[i], world.Cars[j], world);
			}
		}
	}

	return world;
}

void CWorldSimulator::updateCar(const CMyMove& move, CMyCar& car, CCarInfo& carInfo, CMyWorld& world) const
{
	carInfo.LengthwiseUnitVector = { cos(car.Angle), sin(car.Angle) };
	carInfo.CrosswiseUnitVector = { -carInfo.LengthwiseUnitVector.Y, carInfo.LengthwiseUnitVector.X };

	// Проверка дохлости.
	if (car.Durability < 1e-7) {
		if (car.DeadTicks == 0) {
			car.DeadTicks = game.getCarReactivationTimeTicks();
		}
	}
	const bool isDead = car.DeadTicks > 0;
	assert(isDead || car.Durability > 1e-5);
	car.DeadTicks = max(0, car.DeadTicks - 1);
	if (isDead && car.DeadTicks == 0) {
		car.Durability = 1;
	}

	// Нитро.
	if (move.Nitro && !isDead) {
		assert(car.NitroCount > 0 && car.NitroTicks == 0 && car.NitroCooldown == 0);
		car.NitroCount--;
		car.NitroTicks = game.getNitroDurationTicks();
		car.NitroCooldown = game.getUseNitroCooldownTicks();
	}
	const bool isNitro = car.NitroTicks > 0 && !isDead;
	car.NitroTicks = max(0, car.NitroTicks - 1);
	car.NitroCooldown = max(0, car.NitroCooldown - 1);

	// Лужа
	// TODO: Может, лужу тоже перенести в цикл по подтикам?
	if (car.OiledTicks == 0) {
		for (int oilIndex = 0; oilIndex < CMyWorld::MaxOils; oilIndex++) {
			int& oilTicks = world.OilTicks[oilIndex];
			if (oilTicks <= 0) continue;
			const CMyOil& oil = world.Oils[oilIndex];
			if ((car.Position - oil.Position).LengthSquared() <= 150 * 150) {
				car.OiledTicks = min(oilTicks, game.getMaxOiledStateDurationTicks());
				oilTicks -= car.OiledTicks;
			}
		}
	}
	carInfo.IsOiled = car.OiledTicks > 0;
	car.OiledTicks = max(0, car.OiledTicks - 1);

	// Тормоз
	carInfo.IsBrake = move.Brake == 1 && !isDead;

	// Команды на двигатель и на руль.
	const double enginePower = isDead ? 0 : move.Engine;
	const double wheelTurn = isDead ? car.WheelTurn : move.Turn;

	// Обновляем мощность двигателя.
	if (isNitro) {
		car.EnginePower = game.getNitroEnginePowerFactor();
	} else {
		// После окончания нитро надо не забыть обрезать мощность до 1 перед изменением мощности.
		car.EnginePower = limit(car.EnginePower, 1.0);
		car.EnginePower += limit(enginePower - car.EnginePower, game.getCarEnginePowerChangePerTick());
		car.EnginePower = limit(car.EnginePower, 1.0);
	}

	// Вектор ускорения. Будет постоянный для всех итераций физики.
	if (carInfo.IsBrake) {
		carInfo.AccelerationDt = CVec2D(0, 0);
	} else {
		carInfo.AccelerationDt = car.EnginePower >= 0 ?
			carInfo.LengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
			carInfo.LengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;
	}

	// Обновляем угол поворота колёс и базовые (медианные) скорости.
	if (car.MedianAngularSpeed == UndefinedMedianAngularSpeed) {
		// Попытка оценить базовую скорость, если машина была создана из model::Car - там этих данных нет :(
		// К сожалению, такой хак приводит к небольшой потере точности при предсказании руления на первом тике.
		car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * carInfo.LengthwiseUnitVector.DotProduct(car.Speed);
	}
	car.WheelTurn += limit(wheelTurn - car.WheelTurn, game.getCarWheelTurnChangePerTick());
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// Теперь считается базовая скорость на весь текущий тик.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * carInfo.LengthwiseUnitVector.DotProduct(car.Speed);
	car.AngularSpeed += car.MedianAngularSpeed;
}

void CWorldSimulator::moveCar(CCarInfo& carInfo, CMyCar& car) const
{
	// Обновление позиции.
	car.Position += car.Speed * dTime;

	// Обновление скорости.
	car.Speed += carInfo.AccelerationDt;
	car.Speed *= carMovementAirFrictionFactorDt;
	const double frictionLengthwise = limit(car.Speed.DotProduct(carInfo.LengthwiseUnitVector),
		(carInfo.IsBrake && !carInfo.IsOiled) ? carCrosswiseFrictionFactorDt : carLengthwiseFrictionFactorDt);
	const double frictionCrosswise = limit(car.Speed.DotProduct(carInfo.CrosswiseUnitVector),
		carInfo.IsOiled ? carLengthwiseFrictionFactorDt : carCrosswiseFrictionFactorDt);
	car.Speed -= carInfo.LengthwiseUnitVector * frictionLengthwise + carInfo.CrosswiseUnitVector * frictionCrosswise;

	// Обновление угла.
	car.Angle += car.AngularSpeed * dTime;
	carInfo.LengthwiseUnitVector = CVec2D(cos(car.Angle), sin(car.Angle));
	carInfo.CrosswiseUnitVector = CVec2D(carInfo.LengthwiseUnitVector.Y, -carInfo.LengthwiseUnitVector.X);

	// Обновление угловой скорости.
	// Все трения применяются к той части, которая отличается от базовой скорости. Сначала воздушное трение, потом общее трение.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.AngularSpeed *= carRotationAirFrictionFactorDt;
	car.AngularSpeed -= limit(car.AngularSpeed,
		carInfo.IsOiled ? carRotationFrictionFactorDt / 5 : carRotationFrictionFactorDt);
	car.AngularSpeed += car.MedianAngularSpeed;

	normalizeAngle(car.Angle);
	car.RotatedRect = CRotatedRect(car.Position, CMyCar::Width, CMyCar::Height, car.Angle);
}

void CWorldSimulator::moveWasher(CMyWasher& washer) const
{
	washer.Position += washer.Speed * dTime;
}

void CWorldSimulator::moveTire(CMyTire& tire) const
{
	tire.Position += tire.Speed * dTime;
}

void CWorldSimulator::collideTireWithWalls(CMyTire& tire) const
{
	double collisionDeltaSpeed = 0;
	CCollisionInfo collisionInfo;

	// Точки расположены по порядку. Начнём с самого левого угла и левой стенки.
	int cornerIndex = 0;
	CRotatedRect noRotatedRect;
	const CVec2D corners[4] = {
		{ tire.Position.X - CMyTire::Radius, tire.Position.Y },
		{ tire.Position.X, tire.Position.Y - CMyTire::Radius },
		{ tire.Position.X + CMyTire::Radius, tire.Position.Y },
		{ tire.Position.X, tire.Position.Y + CMyTire::Radius }
	};
	CMyTile cornerTile(corners[cornerIndex]);
	double tileLeftX = cornerTile.X * CMyTile::TileSize;
	double tileTopY = cornerTile.Y * CMyTile::TileSize;
	double tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	double tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsLeftOpen()) {
		const double xWall = tileLeftX + CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.X < xWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 1, 0, -xWall };
			collisionInfo.NormalB = { 1, 0 };
			collisionInfo.Depth = xWall - corner.X;
			CLine2D side1, side2;
			if (cornerNext.X < xWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.X < xWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
					tire.InvertedMass, tire.InvertedAngularMass, tire.TireToWallMomentumTransferFactor, tire.TireToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Верхняя стенка.
	cornerIndex = (cornerIndex + 1) % 4;
	cornerTile = CMyTile(corners[cornerIndex]);
	tileLeftX = cornerTile.X * CMyTile::TileSize;
	tileTopY = cornerTile.Y * CMyTile::TileSize;
	tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsTopOpen()) {
		const double yWall = tileTopY + CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.Y < yWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 0, 1, -yWall };
			collisionInfo.NormalB = { 0, 1 };
			collisionInfo.Depth = yWall - corner.Y;
			CLine2D side1, side2;
			if (cornerNext.Y < yWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.Y < yWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
					tire.InvertedMass, tire.InvertedAngularMass, tire.TireToWallMomentumTransferFactor, tire.TireToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Правая стенка.
	cornerIndex = (cornerIndex + 1) % 4;
	cornerTile = CMyTile(corners[cornerIndex]);
	tileLeftX = cornerTile.X * CMyTile::TileSize;
	tileTopY = cornerTile.Y * CMyTile::TileSize;
	tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsRightOpen()) {
		const double xWall = tileRightX - CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.X > xWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 1, 0, -xWall };
			collisionInfo.NormalB = { -1, 0 };
			collisionInfo.Depth = corner.X - xWall;
			CLine2D side1, side2;
			if (cornerNext.X > xWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.X > xWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
					tire.InvertedMass, tire.InvertedAngularMass, tire.TireToWallMomentumTransferFactor, tire.TireToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Нижняя стенка.
	cornerIndex = (cornerIndex + 1) % 4;
	cornerTile = CMyTile(corners[cornerIndex]);
	tileLeftX = cornerTile.X * CMyTile::TileSize;
	tileTopY = cornerTile.Y * CMyTile::TileSize;
	tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsBottomOpen()) {
		const double yWall = tileBottomY - CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.Y > yWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 0, 1, -yWall };
			collisionInfo.NormalB = { 0, -1 };
			collisionInfo.Depth = corner.Y - yWall;
			CLine2D side1, side2;
			if (cornerNext.Y > yWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.Y > yWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
					tire.InvertedMass, tire.InvertedAngularMass, tire.TireToWallMomentumTransferFactor, tire.TireToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Ближайший угол.
	static const double halfTileSize = CMyTile::TileSize;
	const double nearestTileCornerX = (tire.Position.X - tileLeftX < halfTileSize) ? tileLeftX : tileRightX;
	const double nearestTileCornerY = (tire.Position.Y - tileTopY < halfTileSize) ? tileTopY : tileBottomY;
	const CVec2D nearestTileCorner(nearestTileCornerX, nearestTileCornerY);
	const double distanceSqr = (tire.Position - nearestTileCorner).LengthSquared();
	static const double maxDistanceSqr = pow(CMyTile::WallRadius + CMyTire::Radius, 2);
	if (distanceSqr < maxDistanceSqr) {
		CVec2D vectorBA = CVec2D(nearestTileCorner, tire.Position);
		collisionInfo.NormalB = vectorBA;
		collisionInfo.NormalB.Normalize();
		collisionInfo.Point = nearestTileCorner + vectorBA * (CMyTile::WallRadius / (CMyTile::WallRadius + CMyTire::Radius));
		collisionInfo.Depth = CMyTile::WallRadius + CMyTire::Radius - sqrt(distanceSqr);
		resolveCollisionStatic(collisionInfo, tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
			tire.InvertedMass, tire.InvertedAngularMass, tire.TireToWallMomentumTransferFactor, tire.TireToWallSurfaceFrictionFactor);
	}
}

void CWorldSimulator::collideTireWithWashers(CMyTire& tire, CMyWorld& world) const
{
	static const double collisionDistanceSqr = pow(CMyWasher::Radius + CMyTire::Radius, 2);
	bool shouldRemoveInvalidWashers = false;
	for (auto& w : world.Washers) {
		if (!w.IsValid()) {
			break;
		}
		if ((tire.Position - w.Position).LengthSquared() < collisionDistanceSqr) {
			w.Invalidate();
			shouldRemoveInvalidWashers = true;
		}
	}
	if (shouldRemoveInvalidWashers) {
		world.RemoveInvalidWashers();
	}
}

void CWorldSimulator::collideTireWithTires(CMyTire& /*tire*/, CMyWorld& /*world*/) const
{
	//TODO:
}

void CWorldSimulator::collideCarWithWalls(CMyCar& car) const
{
	double collisionDeltaSpeed = 0;
	CCollisionInfo collisionInfo;

	// Точки расположены по порядку. Начнём с самого левого угла и левой стенки.
	double minLeft = INT_MAX;
	int cornerIndex = 0;
	const CVec2D* corners = car.RotatedRect.Corners;
	for (int i = 0; i < 4; i++) {
		if (corners[i].X < minLeft) {
			cornerIndex = i;
			minLeft = corners[i].X;
		}
	}
	CMyTile cornerTile(corners[cornerIndex]);
	double tileLeftX = cornerTile.X * CMyTile::TileSize;
	double tileTopY = cornerTile.Y * CMyTile::TileSize;
	double tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	double tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsLeftOpen()) {
		const double xWall = tileLeftX + CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.X < xWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 1, 0, -xWall };
			collisionInfo.NormalB = { 1, 0 };
			collisionInfo.Depth = xWall - corner.X;
			CLine2D side1, side2;
			if (cornerNext.X < xWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.X < xWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, car.Position, car.Speed, car.AngularSpeed, car.RotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(), car.CarToWallMomentumTransferFactor, car.CarToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Верхняя стенка.
	cornerIndex = (cornerIndex + 1) % 4;
	cornerTile = CMyTile(corners[cornerIndex]);
	tileLeftX = cornerTile.X * CMyTile::TileSize;
	tileTopY = cornerTile.Y * CMyTile::TileSize;
	tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsTopOpen()) {
		const double yWall = tileTopY + CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.Y < yWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 0, 1, -yWall };
			collisionInfo.NormalB = { 0, 1 };
			collisionInfo.Depth = yWall - corner.Y;
			CLine2D side1, side2;
			if (cornerNext.Y < yWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.Y < yWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, car.Position, car.Speed, car.AngularSpeed, car.RotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(), car.CarToWallMomentumTransferFactor, car.CarToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Правая стенка.
	cornerIndex = (cornerIndex + 1) % 4;
	cornerTile = CMyTile(corners[cornerIndex]);
	tileLeftX = cornerTile.X * CMyTile::TileSize;
	tileTopY = cornerTile.Y * CMyTile::TileSize;
	tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsRightOpen()) {
		const double xWall = tileRightX - CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.X > xWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 1, 0, -xWall };
			collisionInfo.NormalB = { -1, 0 };
			collisionInfo.Depth = corner.X - xWall;
			CLine2D side1, side2;
			if (cornerNext.X > xWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.X > xWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, car.Position, car.Speed, car.AngularSpeed, car.RotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(), car.CarToWallMomentumTransferFactor, car.CarToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Нижняя стенка.
	cornerIndex = (cornerIndex + 1) % 4;
	cornerTile = CMyTile(corners[cornerIndex]);
	tileLeftX = cornerTile.X * CMyTile::TileSize;
	tileTopY = cornerTile.Y * CMyTile::TileSize;
	tileRightX = (cornerTile.X + 1) * CMyTile::TileSize;
	tileBottomY = (cornerTile.Y + 1) * CMyTile::TileSize;
	if (!cornerTile.IsBottomOpen()) {
		const double yWall = tileBottomY - CMyTile::WallRadius;
		const CVec2D& corner = corners[cornerIndex];
		if (corner.Y > yWall) {
			const CVec2D& cornerNext = corners[(cornerIndex + 1) % 4];
			const CVec2D& cornerOpposite = corners[(cornerIndex + 2) % 4];
			const CVec2D& cornerPrev = corners[(cornerIndex + 3) % 4];
			const CLine2D wall = { 0, 1, -yWall };
			collisionInfo.NormalB = { 0, -1 };
			collisionInfo.Depth = corner.Y - yWall;
			CLine2D side1, side2;
			if (cornerNext.Y > yWall) {
				side1 = CLine2D::FromPoints(cornerNext, cornerOpposite);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			} else if (cornerPrev.Y > yWall) {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(cornerPrev, cornerOpposite);
			} else {
				side1 = CLine2D::FromPoints(corner, cornerNext);
				side2 = CLine2D::FromPoints(corner, cornerPrev);
			}
			CVec2D intersection1;
			bool hasIntersection1 = wall.GetIntersectionPoint(side1, intersection1);
			CVec2D intersection2;
			bool hasIntersection2 = wall.GetIntersectionPoint(side2, intersection2);
			if (hasIntersection1 && hasIntersection2) {
				collisionInfo.Point = (intersection2 + intersection1) * 0.5;
				resolveCollisionStatic(collisionInfo, car.Position, car.Speed, car.AngularSpeed, car.RotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(), car.CarToWallMomentumTransferFactor, car.CarToWallSurfaceFrictionFactor);
			} else {
				assert(false);
			}
		}
	}

	// Ближайший угол.
	static const double halfTileSize = CMyTile::TileSize;
	const double nearestTileCornerX = (car.Position.X - tileLeftX < halfTileSize) ? tileLeftX : tileRightX;
	const double nearestTileCornerY = (car.Position.Y - tileTopY < halfTileSize) ? tileTopY : tileBottomY;
	const CVec2D nearestTileCorner(nearestTileCornerX, nearestTileCornerY);
	const double distanceSqr = (car.Position - nearestTileCorner).LengthSquared();
	static const double maxDistanceSqr = pow(CMyTile::WallRadius + CMyCar::CircumcircleRadius, 2);
	if (distanceSqr < maxDistanceSqr) {
		for (int i = 0; i < 4; i++) {
			const CVec2D& p1 = corners[i];
			const CVec2D& p2 = corners[(i + 1) % 4];
			if (findLineWithCircleCollision(p1, p2, nearestTileCorner, CMyTile::WallRadius, collisionInfo)) {
				resolveCollisionStatic(collisionInfo, car.Position, car.Speed, car.AngularSpeed, car.RotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(), car.CarToWallMomentumTransferFactor, car.CarToWallSurfaceFrictionFactor);
				break;
			}
		}
	}

	// Обновление жизней.
	static const double durabilityFactor = 0.003;
	static const double durabilityEps = 0.01;
	const double durabilityChange = durabilityFactor * collisionDeltaSpeed;
	if (collisionDeltaSpeed > 0) {
		car.CollisionDeltaSpeed += collisionDeltaSpeed;
		car.CollisionsDetected += 1;
	}
	if (durabilityChange >= durabilityEps) {
		car.Durability = max(0.0, car.Durability - durabilityChange);
	}
}

void CWorldSimulator::collideCarWithWashers(int carId, CMyCar& car, CMyWorld& world) const
{
#define PRECISE_WASHER
#ifdef PRECISE_WASHER
	CCollisionInfo collisionInfo;
	CRotatedRect noRotatedRect;
	double noAngularSpeed = 0;
	double collisionDeltaSpeed = 0;
	bool shouldRemoveInvalidWashers = false;
	for (auto& washer : world.Washers) {
		if (!washer.IsValid()) {
			break;
		}
		static const double minCollisionDistanceSqr = pow(CMyWasher::Radius + CMyCar::CircumcircleRadius, 2);
		if ((washer.Position - car.Position).LengthSquared() > minCollisionDistanceSqr) {
			// Шина находится слишком далеко.
			continue;
		}
		for (int i = 0; i < 4; i++) {
			const CVec2D& p1 = car.RotatedRect.Corners[i];
			const CVec2D& p2 = car.RotatedRect.Corners[(i + 1) % 4];
			if (findLineWithCircleCollision(p1, p2, washer.Position, CMyWasher::Radius, collisionInfo)) {
				if (carId == washer.CarId) {
					// Проверка, что шиной только что выстрелили - если её центр находится внутри машины,
					// или если вектор скорости направлен от машины.
					const CLine2D side = CLine2D::FromPoints(p1, p2);
					if (side.GetSignedDistanceFrom(washer.Position) < 0) {
						// Шина находится внутри машины относительно стороны столкновения.
						continue;
					} else if (washer.Speed.DotProduct(collisionInfo.NormalB) < 0) {
						// Шина находится снаружи, но её скорость направлена от стороны, с которой предполагается столкновение.
						continue;
					}
				}
				resolveCollision(collisionInfo,
					car.Position, car.Speed, car.AngularSpeed, car.RotatedRect,
					washer.Position, washer.Speed, noAngularSpeed, noRotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(),
					washer.InvertedMass, washer.InvertedAngularMass,
					0.5, 0.25);
				// Начисление очков
				const double durabilityChange = min(0.15, car.Durability);
				if (durabilityChange > 0.01) {
					car.Durability = max(0.0, car.Durability - durabilityChange);
					const int washerPlayerId = world.Cars[washer.CarId].PlayerId;
					if (car.PlayerId != washerPlayerId) {
						world.Players[washerPlayerId].Score += static_cast<int>(100 * durabilityChange);
						if (car.Durability == 0.0) {
							world.Players[washerPlayerId].Score += 100;
						}
					}
				}
				washer.Invalidate();
				shouldRemoveInvalidWashers = true;
				break;
			}
		}
	}
	if (shouldRemoveInvalidWashers) {
		world.RemoveInvalidWashers();
	}
#else
	// Неточное, но простое определение.
	bool shouldRemoveInvalidWashers = false;
	for (auto& w : world.Washers) {
		if (!w.IsValid()) {
			break;
		}
		if (w.CarId == carId) {
			continue;
		}
		static const double collisionDistanceSqr = pow(CMyWasher::Radius + CMyCar::HalfHeight, 2);
		if ((w.Position - car.Position).LengthSquared() < collisionDistanceSqr) {
			const double durabilityChange = min(0.15, car.Durability);
			if (durabilityChange > 0.01) {
				car.Durability = max(0.0, car.Durability - durabilityChange);
				const int washerPlayerId = world.Cars[washer.CarId].PlayerId;
				if (car.PlayerId != washerPlayerId) {
					world.Players[washerPlayerId].Score += static_cast<int>(100 * durabilityChange);
					if (car.Durability == 0.0) {
						world.Players[washerPlayerId].Score += 100;
					}
				}
			}
			w.Invalidate();
			shouldRemoveInvalidWashers = true;
			// TODO: Начислять очки игрокам.
		}
	}
	if (shouldRemoveInvalidWashers) {
		world.RemoveInvalidWashers();
	}
#endif
}

void CWorldSimulator::collideCarWithTires(int carId, CMyCar& car, CMyWorld& world) const
{
	CCollisionInfo collisionInfo;
	CRotatedRect noRotatedRect;
	double collisionDeltaSpeed = 0;
	bool shouldRemoveInvalidTires = false;
	for (auto& tire : world.Tires) {
		if (!tire.IsValid()) {
			break;
		}
		static const double minCollisionDistanceSqr = pow(CMyTire::Radius + CMyCar::CircumcircleRadius, 2);
		if ((tire.Position - car.Position).LengthSquared() > minCollisionDistanceSqr) {
			// Шина находится слишком далеко.
			continue;
		}
		for (int i = 0; i < 4; i++) {
			const CVec2D& p1 = car.RotatedRect.Corners[i];
			const CVec2D& p2 = car.RotatedRect.Corners[(i + 1) % 4];
			if (findLineWithCircleCollision(p1, p2, tire.Position, CMyTire::Radius, collisionInfo)) {
				if (carId == tire.CarId) {
					// Проверка, что шиной только что выстрелили - если её центр находится внутри машины,
					// или если вектор скорости направлен от машины.
					const CLine2D side = CLine2D::FromPoints(p1, p2);
					if (side.GetSignedDistanceFrom(tire.Position) < 0) {
						// Шина находится внутри машины относительно стороны столкновения.
						continue;
					} else if(tire.Speed.DotProduct(collisionInfo.NormalB) < 0) {
						// Шина находится снаружи, но её скорость направлена от стороны, с которой предполагается столкновение.
						continue;
					}
				}
				resolveCollision(collisionInfo,
					car.Position, car.Speed, car.AngularSpeed, car.RotatedRect,
					tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(),
					tire.InvertedMass, tire.InvertedAngularMass,
					car.CarToTireMomentumTransferFactor, car.CarToTireSurfaceFrictionFactor);
				// Начисление очков
				const double durabilityChange = min(0.35 * collisionDeltaSpeed / 60.0, car.Durability);
				if (durabilityChange > 0.01) {
					car.Durability = max(0.0, car.Durability - durabilityChange);
					const int tirePlayerId = world.Cars[tire.CarId].PlayerId;
					if (car.PlayerId != tirePlayerId) {
						world.Players[tirePlayerId].Score += static_cast<int>(100 * durabilityChange);
						if (car.Durability == 0.0) {
							world.Players[tirePlayerId].Score += 100;
						}
					}
				}
				if (tire.Speed.Length() < minTireSpeed) {
					tire.Invalidate();
					shouldRemoveInvalidTires = true;
				}
				break;
			}
		}
	}
	if (shouldRemoveInvalidTires) {
		world.RemoveInvalidTires();
	}
}

void CWorldSimulator::collideCarWithBonuses(CMyCar& car, CMyWorld& world) const
{
//#define PRECISE_BONUSES
#undef PRECISE_BONUSES
#ifdef PRECISE_BONUSES
	//TODO
#else
	for (int i = 0; i < CMyWorld::MaxBonuses; i++) {
		if (!world.BonusExist[i]) continue;
		const double distanceSqr = (world.Bonuses[i].Position - car.Position).LengthSquared();
		static const double minDistanceSqr = pow(CMyCar::CircumcircleRadius + CMyBonus::Size / 2, 2);
		if (distanceSqr > minDistanceSqr) {
			continue;
		}
		static const double sureDistanceSqr = pow(CMyCar::HalfHeight + CMyBonus::Size / 2, 2);
		bool pickedUp = distanceSqr < sureDistanceSqr;
		if (!pickedUp) {
			for (const auto& c : car.RotatedRect.Corners) {
				const double cornerDistanceSqr = (world.Bonuses[i].Position - c).LengthSquared();
				static const double sureCornerDistanceSqr = pow(CMyBonus::Size / 2, 2);
				if (cornerDistanceSqr < sureCornerDistanceSqr) {
					pickedUp = true;
					break;
				}
			}
		}
		if (pickedUp) {
			world.BonusExist[i] = false;
			switch (world.Bonuses[i].Type) {
			case model::BonusType::REPAIR_KIT:
				car.Durability = 1;
				break;
			case model::BonusType::AMMO_CRATE:
				car.ProjectilesCount += 1;
				break;
			case model::BonusType::NITRO_BOOST:
				car.NitroCount += 1;
				break;
			case model::BonusType::OIL_CANISTER:
				car.OilCount += 1;
				break;
			case model::BonusType::PURE_SCORE:
				world.Players[car.PlayerId].Score += 100;
				break;
			default:
				assert(false);
			}
		}
	}
#endif
}

void CWorldSimulator::collideCarWithCar(CMyCar& carA, CMyCar& carB, CMyWorld& world) const
{
	static const double minCollisionDistanceSqr = pow(CMyCar::CircumcircleRadius + CMyCar::CircumcircleRadius, 2);
	if ((carA.Position - carB.Position).LengthSquared() < minCollisionDistanceSqr) {
		CCollisionInfo collisionInfo;
		double collisionDeltaSpeed = 0;
		if (findCarWithCarCollision(carA, carB, collisionInfo)) {
			resolveCollision(collisionInfo,
				carA.Position, carA.Speed, carA.AngularSpeed, carA.RotatedRect,
				carB.Position, carB.Speed, carB.AngularSpeed, carB.RotatedRect, collisionDeltaSpeed,
				carA.GetInvertedMass(), carA.GetInvertedAngularMass(),
				carB.GetInvertedMass(), carB.GetInvertedAngularMass(),
				CMyCar::CarToCarMomentumTransferFactor, CMyCar::CarToCarSurfaceFrictionFactor);

			// TODO: проверить повреждения при машинах разной массы
			static const double durabilityFactor = 0.003;
			const double durabilityChangeA = min(durabilityFactor * collisionDeltaSpeed, carA.Durability);
			if (durabilityChangeA > 0.01) {
				carA.Durability = max(0.0, carA.Durability - durabilityChangeA);
				const int otherPlayerId = carB.PlayerId;
				if (carA.PlayerId != otherPlayerId) {
					world.Players[otherPlayerId].Score += static_cast<int>(100 * durabilityChangeA);
					if (carB.Durability == 0.0) {
						world.Players[otherPlayerId].Score += 100;
					}
				}
			}

			const double durabilityChangeB = min(durabilityFactor * collisionDeltaSpeed, carB.Durability);
			if (durabilityChangeB > 0.01) {
				carB.Durability = max(0.0, carB.Durability - durabilityChangeB);
				const int otherPlayerId = carA.PlayerId;
				if (carB.PlayerId != otherPlayerId) {
					world.Players[otherPlayerId].Score += static_cast<int>(100 * durabilityChangeB);
					if (carB.Durability == 0.0) {
						world.Players[otherPlayerId].Score += 100;
					}
				}
			}
		}
	}
}

bool CWorldSimulator::findLineWithCircleCollision(const CVec2D& point1A, const CVec2D& point2A,
	const CVec2D& positionB, double radiusB,
	CCollisionInfo& collisionInfo) const
{
	CLine2D lineA = CLine2D::FromPoints(point1A, point2A);

	const double distanceFromB = lineA.GetDistanceFrom(positionB);
	if (distanceFromB > radiusB) {
		return false;
	}

	const double leftA = min(point1A.X, point2A.X);
	const double topA = min(point1A.Y, point2A.Y);
	const double rightA = max(point1A.X, point2A.X);
	const double bottomA = max(point1A.Y, point2A.Y);

	CVec2D projectionOfB = lineA.GetProjectionOf(positionB);

	bool projectionOfBBelongsToA = (projectionOfB.X > leftA - epsilon)
		&& (projectionOfB.X < rightA + epsilon)
		&& (projectionOfB.Y > topA - epsilon)
		&& (projectionOfB.Y < bottomA + epsilon);

	if (projectionOfBBelongsToA) {
		CVec2D collisionNormalB;

		if (distanceFromB >= epsilon) {
			collisionNormalB = CVec2D(positionB, projectionOfB);
			collisionNormalB.Normalize();
		} else {
			//assert(false);
			return false;
		}
		collisionInfo.NormalB = collisionNormalB;
		collisionInfo.Point = projectionOfB;
		collisionInfo.Depth = radiusB - distanceFromB;
		return true;
	}

	double distanceToPoint1A = (positionB - point1A).Length();
	double distanceToPoint2A = (positionB - point2A).Length();

	CVec2D nearestPointA;
	double distanceToNearestPointA = INT_MAX;
	if (distanceToPoint1A < distanceToPoint2A) {
		nearestPointA = point1A;
		distanceToNearestPointA = distanceToPoint1A;
	} else {
		nearestPointA = point2A;
		distanceToNearestPointA = distanceToPoint2A;
	}

	if (distanceToNearestPointA > radiusB) {
		return false;
	}

	collisionInfo.Point = nearestPointA;
	collisionInfo.NormalB = CVec2D(positionB, nearestPointA);
	collisionInfo.NormalB.Normalize();
	collisionInfo.Depth = radiusB - distanceToNearestPointA;
	return true;
}

bool CWorldSimulator::findCarWithCarCollision(const CMyCar& carA, const CMyCar& carB,
	CCollisionInfo& collisionInfo) const
{
	CCollisionInfo collisionInfoA;
	if (!findCarWithCarCollisionPartial(carA, carB, collisionInfoA)) {
		return false;
	}

	CCollisionInfo collisionInfoB;
	if (!findCarWithCarCollisionPartial(carB, carA, collisionInfoB)) {
		return false;
	}

	if (collisionInfoB.Depth < collisionInfoA.Depth) {
		collisionInfo.Point = collisionInfoB.Point;
		collisionInfo.Depth = collisionInfoB.Depth;
		collisionInfo.NormalB = -collisionInfoB.NormalB;
		return true;
	} else {
		collisionInfo = collisionInfoA;
		return true;
	}
}

bool CWorldSimulator::findCarWithCarCollisionPartial(const CMyCar& carA, const CMyCar& carB,
	CCollisionInfo& collisionInfo) const
{
	double minDepth = INT_MAX;
	CVec2D bestIntersectionPoint;
	CVec2D bestCollisionNormalB;

	for (int pointAIndex = 0; pointAIndex < 4; ++pointAIndex) {
		const CVec2D& point1A = carA.RotatedRect.Corners[pointAIndex];
		const CVec2D& point2A = carA.RotatedRect.Corners[(pointAIndex + 1) % 4];
		CLine2D lineA = CLine2D::FromPoints(point1A, point2A);

		if (lineA.GetSignedDistanceFrom(carA.Position) > -epsilon) {
			//throw new IllegalStateException(String.format("%s of %s is too small, " +
			//	"does not represent a convex polygon, or its points are going in wrong order.",
			//	Form.toString(bodyA.getForm()), bodyA
			//	));
			assert(false);
		}

		double minDistanceFromB = INT_MAX;
		CVec2D intersectionPoint;
		CVec2D collisionNormalB;

		for (int pointBIndex = 0; pointBIndex < 4; ++pointBIndex) {
			const CVec2D& pointB = carB.RotatedRect.Corners[pointBIndex];
			double distanceFromPointB = lineA.GetSignedDistanceFrom(pointB);

			if (distanceFromPointB < minDistanceFromB) {
				minDistanceFromB = distanceFromPointB;
				intersectionPoint = pointB;
				collisionNormalB = -lineA.GetUnitNormalFrom(carA.Position);
			}
		}

		if (minDistanceFromB > 0.0) {
			return false;
		}

		double depth = -minDistanceFromB;
		if (depth < minDepth) {
			minDepth = depth;
			bestIntersectionPoint = intersectionPoint;
			bestCollisionNormalB = collisionNormalB;
		}
	}

	if (minDepth == INT_MAX) {
		return false;
	}
	collisionInfo.Point = bestIntersectionPoint;
	collisionInfo.NormalB = bestCollisionNormalB;
	collisionInfo.Depth = minDepth;
	return true;
}


void CWorldSimulator::resolveCollisionStatic(
	const CCollisionInfo& collisionInfo,
	CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRectA, double& collisionDeltaSpeed,
	double invertedMassA, double invertedAngularMassA,
	double momentumTransferFactorAB, double surfaceFrictionFactorAB) const
{
	CVec3D collisionNormalB(collisionInfo.NormalB);
	CVec3D vectorAC(collisionInfo.Point - positionA);
	CVec3D angularVelocityPartAC = CVec3D(angularSpeedA).Cross(vectorAC);
	CVec3D velocityAC = angularVelocityPartAC + CVec3D(speedA);
	CVec3D relativeVelocityC = velocityAC;

	const double normalRelativeVelocityLengthC = -relativeVelocityC.DotProduct(collisionNormalB);
	if (normalRelativeVelocityLengthC > -epsilon) {
		resolveImpactStatic(vectorAC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA,
			invertedMassA, invertedAngularMassA, momentumTransferFactorAB);
		resolveSurfaceFrictionStatic(vectorAC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA,
			invertedMassA, invertedAngularMassA, surfaceFrictionFactorAB);
		collisionDeltaSpeed += normalRelativeVelocityLengthC;
	}
	pushBackBodiesStatic(collisionInfo.NormalB, collisionInfo.Depth, positionA, rotatedRectA);

}

void CWorldSimulator::resolveImpactStatic(
	const CVec3D& vectorAC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
	CVec2D& speedA, double& angularSpeedA,
	double invertedMassA, double invertedAngularMassA,
	double momentumTransferFactorAB) const
{
	CVec3D denominatorPartA = vectorAC.Cross(collisionNormalB);
	denominatorPartA *= invertedAngularMassA;
	denominatorPartA = denominatorPartA.Cross(vectorAC);
	const double denominator = invertedMassA + collisionNormalB.DotProduct(denominatorPartA);
	const double impulseChange = -(1 + momentumTransferFactorAB) * relativeVelocityC.DotProduct(collisionNormalB) / denominator;
	if (impulseChange < epsilon) {
		return;
	}

	CVec3D velocityChangeA = collisionNormalB * (impulseChange * invertedMassA);
	speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
	CVec3D angularVelocityChangeA = vectorAC.Cross(collisionNormalB * impulseChange) * invertedAngularMassA;
	angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;
}

void CWorldSimulator::resolveSurfaceFrictionStatic(
	const CVec3D& vectorAC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
	CVec2D& speedA, double& angularSpeedA,
	double invertedMassA, double invertedAngularMassA,
	double surfaceFrictionFactorAB) const
{
	CVec3D tangent = relativeVelocityC - (collisionNormalB * (relativeVelocityC.DotProduct(collisionNormalB)));
	if (tangent.LengthSquared() < epsilon * epsilon) {
		return;
	}

	tangent *= 1.0 / tangent.Length();
	static const double sqrt2 = sqrt(2);
	const double surfaceFrictionFactorABSqrt = sqrt2 * sqrt(surfaceFrictionFactorAB);
	const double surfaceFriction = surfaceFrictionFactorABSqrt * abs(relativeVelocityC.DotProduct(collisionNormalB)) / relativeVelocityC.Length();
	if (surfaceFriction < epsilon) {
		return;
	}

	CVec3D denominatorPartA = vectorAC.Cross(tangent);
	denominatorPartA *= invertedAngularMassA;
	denominatorPartA = denominatorPartA.Cross(vectorAC);
	const double denominator = invertedMassA + tangent.DotProduct(denominatorPartA);
	const double impulseChange = -surfaceFriction * relativeVelocityC.DotProduct(tangent) / denominator;
	if (abs(impulseChange) < epsilon) {
		return;
	}

	CVec3D velocityChangeA = tangent * (impulseChange * invertedMassA);
	speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
	CVec3D angularVelocityChangeA = vectorAC.Cross(tangent * impulseChange) * invertedAngularMassA;
	angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;
}

void CWorldSimulator::pushBackBodiesStatic(
	const CVec2D& collisionNormalB2D, double depth,
	CVec2D& positionA, CRotatedRect& rotatedRect) const
{
	const CVec2D shift = collisionNormalB2D * (depth + epsilon);
	positionA += shift;
	for (auto& c : rotatedRect.Corners) {
		c += shift;
	}
}

void CWorldSimulator::resolveCollision(
	const CCollisionInfo& collisionInfo,
	CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRectA,
	CVec2D& positionB, CVec2D& speedB, double& angularSpeedB, CRotatedRect& rotatedRectB, double& collisionDeltaSpeed,
	double invertedMassA, double invertedAngularMassA, double invertedMassB, double invertedAngularMassB,
	double momentumTransferFactorAB, double surfaceFrictionFactorAB) const
{
	CVec3D collisionNormalB(collisionInfo.NormalB);
	CVec3D vectorAC(collisionInfo.Point - positionA);
	CVec3D vectorBC(collisionInfo.Point - positionB);
	CVec3D angularVelocityPartAC = CVec3D(angularSpeedA).Cross(vectorAC);
	CVec3D angularVelocityPartBC = CVec3D(angularSpeedB).Cross(vectorBC);
	CVec3D velocityAC = angularVelocityPartAC + CVec3D(speedA);
	CVec3D velocityBC = angularVelocityPartBC + CVec3D(speedB);
	CVec3D relativeVelocityC = velocityAC - velocityBC;

	const double normalRelativeVelocityLengthC = -relativeVelocityC.DotProduct(collisionNormalB);
	if (normalRelativeVelocityLengthC > -epsilon) {
		resolveImpact(vectorAC, vectorBC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA, speedB, angularSpeedB,
			invertedMassA, invertedAngularMassA, invertedMassB, invertedAngularMassB, momentumTransferFactorAB);
		resolveSurfaceFriction(vectorAC, vectorBC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA, speedB, angularSpeedB,
			invertedMassA, invertedAngularMassA, invertedMassB, invertedAngularMassB, surfaceFrictionFactorAB);
		collisionDeltaSpeed += normalRelativeVelocityLengthC;
	}
	pushBackBodies(collisionInfo.NormalB, collisionInfo.Depth, positionA, rotatedRectA, positionB, rotatedRectB);
}

void CWorldSimulator::resolveImpact(
	const CVec3D& vectorAC, const CVec3D& vectorBC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
	CVec2D& speedA, double& angularSpeedA,
	CVec2D& speedB, double& angularSpeedB,
	double invertedMassA, double invertedAngularMassA, double invertedMassB, double invertedAngularMassB,
	double momentumTransferFactorAB) const
{
	CVec3D denominatorPartA = vectorAC.Cross(collisionNormalB);
	denominatorPartA *= invertedAngularMassA;
	denominatorPartA = denominatorPartA.Cross(vectorAC);
	CVec3D denominatorPartB = vectorBC.Cross(collisionNormalB);
	denominatorPartB *= invertedAngularMassB;
	denominatorPartB = denominatorPartB.Cross(vectorBC);
	const double denominator = invertedMassA + invertedMassB + collisionNormalB.DotProduct(denominatorPartA + denominatorPartB);
	const double impulseChange = -(1 + momentumTransferFactorAB) * relativeVelocityC.DotProduct(collisionNormalB) / denominator;
	if (impulseChange < epsilon) {
		return;
	}

	CVec3D velocityChangeA = collisionNormalB * (impulseChange * invertedMassA);
	speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
	CVec3D angularVelocityChangeA = vectorAC.Cross(collisionNormalB * impulseChange) * invertedAngularMassA;
	angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;

	CVec3D velocityChangeB = collisionNormalB * (impulseChange * invertedMassB);
	speedB = { speedB.X - velocityChangeB.X, speedB.Y - velocityChangeB.Y };
	CVec3D angularVelocityChangeB = vectorBC.Cross(collisionNormalB * impulseChange) * invertedAngularMassB;
	angularSpeedB = angularSpeedB - angularVelocityChangeB.Z;
}

void CWorldSimulator::resolveSurfaceFriction(
	const CVec3D& vectorAC, const CVec3D& vectorBC, const CVec3D& collisionNormalB, const CVec3D& relativeVelocityC,
	CVec2D& speedA, double& angularSpeedA,
	CVec2D& speedB, double& angularSpeedB,
	double invertedMassA, double invertedAngularMassA, double invertedMassB, double invertedAngularMassB,
	double surfaceFrictionFactorAB) const
{
	CVec3D tangent = relativeVelocityC - (collisionNormalB * (relativeVelocityC.DotProduct(collisionNormalB)));
	if (tangent.LengthSquared() < epsilon * epsilon) {
		return;
	}

	tangent *= 1.0 / tangent.Length();
	static const double sqrt2 = sqrt(2);
	const double surfaceFrictionFactorABSqrt = sqrt2 * sqrt(surfaceFrictionFactorAB);
	const double surfaceFriction = surfaceFrictionFactorABSqrt * abs(relativeVelocityC.DotProduct(collisionNormalB)) / relativeVelocityC.Length();
	if (surfaceFriction < epsilon) {
		return;
	}

	CVec3D denominatorPartA = vectorAC.Cross(tangent);
	denominatorPartA *= invertedAngularMassA;
	denominatorPartA = denominatorPartA.Cross(vectorAC);
	CVec3D denominatorPartB = vectorBC.Cross(tangent);
	denominatorPartB *= invertedAngularMassB;
	denominatorPartB = denominatorPartB.Cross(vectorBC);
	const double denominator = invertedMassA + invertedMassB + tangent.DotProduct(denominatorPartA + denominatorPartB);
	const double impulseChange = -surfaceFriction * relativeVelocityC.DotProduct(tangent) / denominator;
	if (abs(impulseChange) < epsilon) {
		return;
	}

	CVec3D velocityChangeA = tangent * (impulseChange * invertedMassA);
	speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
	CVec3D angularVelocityChangeA = vectorAC.Cross(tangent * impulseChange) * invertedAngularMassA;
	angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;

	CVec3D velocityChangeB = tangent * (impulseChange * invertedMassB);
	speedB = { speedB.X - velocityChangeB.X, speedB.Y - velocityChangeB.Y };
	CVec3D angularVelocityChangeB = vectorBC.Cross(tangent * impulseChange) * invertedAngularMassB;
	angularSpeedB = angularSpeedB - angularVelocityChangeB.Z;
}

void CWorldSimulator::pushBackBodies(
	const CVec2D& collisionNormalB2D, double depth,
	CVec2D& positionA, CRotatedRect& rotatedRectA,
	CVec2D& positionB, CRotatedRect& rotatedRectB) const
{
	CVec2D normalOffset = collisionNormalB2D * (0.5 * depth + epsilon);
	positionA += normalOffset;
	for (auto& c : rotatedRectA.Corners) {
		c += normalOffset;
	}
	positionB -= normalOffset;
	for (auto& c : rotatedRectB.Corners) {
		c -= normalOffset;
	}
}
