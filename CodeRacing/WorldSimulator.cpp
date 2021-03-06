#include "WorldSimulator.h"

#ifdef LOGGING
#undef NDEBUG
#endif

#include <algorithm>
#include "assert.h"
#include "Arc2D.h"
#include "Line2D.h"
#include "Log.h"
#include "MyTile.h"
#include "Tools.h"

using namespace std;

static const double epsilon = 1e-7;
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
	if (_subtickCount != subtickCount) {
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
}

void CWorldSimulator::SetOptions(bool _stopCollisions, bool _ignoreProjectiles, bool _ignoreOtherCars)
{
	stopCollisions = _stopCollisions;
	ignoreProjectiles = _ignoreProjectiles;
	ignoreOtherCars = _ignoreOtherCars;
}

void CWorldSimulator::Simulate(CMyWorld& world, const CMyMove moves[CMyWorld::MaxCars]) const
{
	CCarInfo carInfos[CMyWorld::MaxCars];
	for (int i = 0; i < CMyWorld::MaxCars; i++) {
		if (!world.Cars[i].IsValid() || world.Cars[i].IsFinished) {
			continue;
		}
		updateCar(moves[i], world.Cars[i], carInfos[i], world);
	}

	// TODO: ������� � ��������� �����.
	for (int i = 0; i < CMyWorld::MaxOils; i++) {
		world.OilTicks[i] = max(0, world.OilTicks[i] - 1);
	}

	for (int subtick = 0; subtick < subtickCount; subtick++) {
		// ������� ������
		for (int i = 0; i < CMyWorld::MaxCars; i++) {
			if (ignoreOtherCars && i > 0) {
				break;
			}
			if (!world.Cars[i].IsValid() || world.Cars[i].IsFinished) {
				continue;
			}
			moveCar(carInfos[i], world.Cars[i]);
		}


		if(!ignoreProjectiles) {
			// ������� �������
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

			// �������� �������� �� ������� � ���� � ������
			bool shouldRemoveInvalidTires = false;
			for (int i = 0; i < CMyWorld::MaxTires; i++) {
				if (!world.Tires[i].IsValid()) {
					break;
				}
				collideTireWithWalls(world.Tires[i]);
				collideTireWithWashers(world.Tires[i], world);
				for (int j = i + 1; j < CMyWorld::MaxTires; j++) {
					if (!world.Tires[j].IsValid()) {
						break;
					}
					collideTireWithTire(world.Tires[i], world.Tires[j]);
				}
				if (world.Tires[i].Speed.Length() < minTireSpeed) {
					world.Tires[i].Invalidate();
					shouldRemoveInvalidTires = true;
				}
			}
			if (shouldRemoveInvalidTires) {
				world.RemoveInvalidTires();
			}
		}

		// �������� �����
		for (int i = 0; i < CMyWorld::MaxCars; i++) {
			if (ignoreOtherCars && i > 0) {
				break;
			}
			if (!world.Cars[i].IsValid() || world.Cars[i].IsFinished) {
				continue;
			}
			collideCarWithWalls(world.Cars[i]);
			collideCarWithBonuses(world.Cars[i], world);
			if (!ignoreProjectiles) {
				collideCarWithWashers(i, world.Cars[i], world);
				collideCarWithTires(i, world.Cars[i], world);
			}
			if (!ignoreOtherCars) {
				for (int j = i + 1; j < CMyWorld::MaxCars; j++) {
					if (!world.Cars[i].IsValid() || world.Cars[j].IsFinished) {
						continue;
					}
					collideCarWithCar(world.Cars[i], world.Cars[j], world);
				}
			}
		}
	}

	// �������� �� �����.
	// TODO: ������� � ��������� �����
	for(auto& car : world.Cars) {
		if (!car.IsValid() || car.IsFinished) {
			continue;
		}
		if (car.OiledTicks == 0) {
			for (int oilIndex = 0; oilIndex < CMyWorld::MaxOils; oilIndex++) {
				int& oilTicks = world.OilTicks[oilIndex];
				if (oilTicks <= 0) continue;
				const CMyOil& oil = world.Oils[oilIndex];
				if ((car.Position - oil.Position).LengthSquared() <= pow(CMyOil::Radius, 2)) {
					car.OiledTicks = min(oilTicks, game.getMaxOiledStateDurationTicks()) - 1;
					oilTicks -= car.OiledTicks;
				}
			}
		}
	}

	// NextWaypointIndex � ����� ����.
	// TODO: ������� � ��������� �����
	for (auto& car : world.Cars) {
		const int carTileX = static_cast<int>(car.Position.X / CMyTile::TileSize);
		const int carTileY = static_cast<int>(car.Position.Y / CMyTile::TileSize);
		const CMyTile& nextWPTile = CMyWorld::WaypointTiles[car.NextWaypointIndex];
		if (carTileX == nextWPTile.X && carTileY == nextWPTile.Y) {
			car.NextWaypointIndex = (car.NextWaypointIndex + 1) % CMyWorld::WaypointTiles.size();
			if (car.NextWaypointIndex == 1) {
				car.LapsCount += 1;
			}
		}
	}
}

void CWorldSimulator::updateCar(const CMyMove& move, CMyCar& car, CCarInfo& carInfo, CMyWorld& world) const
{
	carInfo.LengthwiseUnitVector = { cos(car.Angle), sin(car.Angle) };
	carInfo.CrosswiseUnitVector = { -carInfo.LengthwiseUnitVector.Y, carInfo.LengthwiseUnitVector.X };

	// ��������.
	if (move.Shoot && car.ProjectilesCount > 0 && car.ProjectileCooldown == 0) {
		if (car.Type == 0) {
			int i = 0;
			while (i < CMyWorld::MaxWashers && world.Washers[i].IsValid()) i++;
			for (int j = -1; j <= 1; j++) {
				if (i < CMyWorld::MaxWashers) {
					world.Washers[i].CarId = 0;
					world.Washers[i].Position = car.Position;
					const double angle = car.Angle + j * game.getSideWasherAngle();
					world.Washers[i].Speed = CVec2D(cos(angle), sin(angle)) * game.getWasherInitialSpeed();
					i++;
				} else {
					CLog::Instance().Stream() << "Warning! Too many washers" << endl;
				}
			}
		} else {
			int i = 0;
			while (i < CMyWorld::MaxTires && world.Tires[i].IsValid()) i++;
			if (i < CMyWorld::MaxTires) {
				world.Tires[i].CarId = 0;
				world.Tires[i].Position = car.Position;
				world.Tires[i].Speed = CVec2D(cos(car.Angle), sin(car.Angle)) * game.getTireInitialSpeed();
				world.Tires[i].AngularSpeed = 0;
				world.Tires[i].CollisionsCount = 0;
				i++;
			} else {
				CLog::Instance().Stream() << "Warning! Too many tires" << endl;
			}
		}
		car.ProjectilesCount -= 1;
		car.ProjectileCooldown = game.getThrowProjectileCooldownTicks();
	}
	car.ProjectileCooldown = max(0, car.ProjectileCooldown - 1);

	// ��������� ����.
	if (move.Oil && car.OilCount > 0 && car.OilCooldown == 0) {
		int i = 0;
		while (i < CMyWorld::MaxOils && world.OilTicks[i] > 0) i++;
		if (i < CMyWorld::MaxOils) {
			CVec2D oilOffset(CMyCar::HalfWidth + game.getOilSlickInitialRange() + CMyOil::Radius, 0);
			oilOffset.Rotate(car.Angle);
			world.Oils[i].Position = car.Position - oilOffset;
			world.OilTicks[i] = game.getOilSlickLifetime();
		} else {
			CLog::Instance().Stream() << "Warning! Too many oils" << endl;
		}
		car.OilCount -= 1;
		car.OilCooldown = game.getSpillOilCooldownTicks();
	}
	car.OilCooldown = max(0, car.OilCooldown - 1);

	// �������� ��������.
	if (car.Durability <= 0) {
		if (car.DeadTicks == 0) {
			car.DeadTicks = game.getCarReactivationTimeTicks();
		}
	}
	const bool isDead = car.DeadTicks > 0;
	assert(isDead || car.Durability > 0);
	car.DeadTicks = max(0, car.DeadTicks - 1);
	if (isDead && car.DeadTicks == 0) {
		car.Durability = 1;
	}

	// �����.
	if (move.Nitro && !isDead) {
		assert(car.NitroCount > 0 && car.NitroTicks == 0 && car.NitroCooldown == 0);
		car.NitroCount--;
		car.NitroTicks = game.getNitroDurationTicks();
		car.NitroCooldown = game.getUseNitroCooldownTicks();
	}
	const bool isNitro = car.NitroTicks > 0 && !isDead;
	car.NitroTicks = max(0, car.NitroTicks - 1);
	car.NitroCooldown = max(0, car.NitroCooldown - 1);

	carInfo.IsOiled = car.OiledTicks > 0;
	car.OiledTicks = max(0, car.OiledTicks - 1);

	// ������
	carInfo.IsBrake = move.Brake == 1 && !isDead;

	// ������� �� ��������� � �� ����.
	const double enginePower = isDead ? 0 : move.Engine;
	const double wheelTurn = isDead ? car.WheelTurn : move.Turn;

	// ��������� �������� ���������.
	if (isNitro) {
		car.EnginePower = game.getNitroEnginePowerFactor();
	} else {
		// ����� ��������� ����� ���� �� ������ �������� �������� �� 1 ����� ���������� ��������.
		car.EnginePower = limit(car.EnginePower, 1.0);
		car.EnginePower += limit(enginePower - car.EnginePower, game.getCarEnginePowerChangePerTick());
		car.EnginePower = limit(car.EnginePower, 1.0);
	}

	// ������ ���������. ����� ���������� ��� ���� �������� ������.
	if (carInfo.IsBrake) {
		carInfo.AccelerationDt = CVec2D(0, 0);
	} else {
		carInfo.AccelerationDt = car.EnginePower >= 0 ?
			carInfo.LengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
			carInfo.LengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;
	}

	// ��������� ���� �������� ���� � ������� (���������) ��������.
	if (car.MedianAngularSpeed == UndefinedMedianAngularSpeed) {
		// ������� ������� ������� ��������, ���� ������ ���� ������� �� model::Car - ��� ���� ������ ��� :(
		// � ���������, ����� ��� �������� � ��������� ������ �������� ��� ������������ ������� �� ������ ����.
		car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * carInfo.LengthwiseUnitVector.DotProduct(car.Speed);
	}
	car.WheelTurn += limit(wheelTurn - car.WheelTurn, game.getCarWheelTurnChangePerTick());
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// ������ ��������� ������� �������� �� ���� ������� ���.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * carInfo.LengthwiseUnitVector.DotProduct(car.Speed);
	car.AngularSpeed += car.MedianAngularSpeed;
}

void CWorldSimulator::moveCar(CCarInfo& carInfo, CMyCar& car) const
{
	// ���������� �������.
	car.Position += car.Speed * dTime;

	// ���������� ��������.
	car.Speed += carInfo.AccelerationDt;
	car.Speed *= carMovementAirFrictionFactorDt;
	const double frictionLengthwise = limit(car.Speed.DotProduct(carInfo.LengthwiseUnitVector),
		(carInfo.IsBrake && !carInfo.IsOiled) ? carCrosswiseFrictionFactorDt : carLengthwiseFrictionFactorDt);
	const double frictionCrosswise = limit(car.Speed.DotProduct(carInfo.CrosswiseUnitVector),
		carInfo.IsOiled ? carLengthwiseFrictionFactorDt : carCrosswiseFrictionFactorDt);
	car.Speed -= carInfo.LengthwiseUnitVector * frictionLengthwise + carInfo.CrosswiseUnitVector * frictionCrosswise;

	// ���������� ����.
	car.Angle += car.AngularSpeed * dTime;
	carInfo.LengthwiseUnitVector = CVec2D(cos(car.Angle), sin(car.Angle));
	carInfo.CrosswiseUnitVector = CVec2D(-carInfo.LengthwiseUnitVector.Y, carInfo.LengthwiseUnitVector.X);

	// ���������� ������� ��������.
	// ��� ������ ����������� � ��� �����, ������� ���������� �� ������� ��������. ������� ��������� ������, ����� ����� ������.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.AngularSpeed *= carRotationAirFrictionFactorDt;
	car.AngularSpeed -= limit(car.AngularSpeed,
		carInfo.IsOiled ? carRotationFrictionFactorDt / 5 : carRotationFrictionFactorDt);
	car.AngularSpeed += car.MedianAngularSpeed;

	normalizeAngle(car.Angle);
	CVec2D halfWidthVector = carInfo.LengthwiseUnitVector;
	halfWidthVector *= CMyCar::HalfWidth;
	CVec2D halfHeightVector = carInfo.CrosswiseUnitVector;
	halfHeightVector *= CMyCar::HalfHeight;
	car.RotatedRect.Corners[0] = car.Position + halfWidthVector + halfHeightVector;
	car.RotatedRect.Corners[1] = car.Position - halfWidthVector + halfHeightVector;
	car.RotatedRect.Corners[2] = car.Position - halfWidthVector - halfHeightVector;
	car.RotatedRect.Corners[3] = car.Position + halfWidthVector - halfHeightVector;
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

	// ����� ����������� �� �������. ������ � ������ ������ ���� � ����� ������.
	int cornerIndex = 0;
	CRotatedRect noRotatedRect;
	const CVec2D corners[4] = {
		{ tire.Position.X - CMyTire::Radius, tire.Position.Y },
		{ tire.Position.X, tire.Position.Y - CMyTire::Radius },
		{ tire.Position.X + CMyTire::Radius, tire.Position.Y },
		{ tire.Position.X, tire.Position.Y + CMyTire::Radius }
	};

	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileLeftX = cornerTileX * CMyTile::TileSize;
		if (cornerTileX == 0 || !CMyTile::IsLeftOpen(cornerTileX, cornerTileY)) {
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
					tire.CollisionsCount += 1;
				} else {
					assert(false);
				}
			}
		}
	}

	// ������� ������.
	cornerIndex = (cornerIndex + 1) % 4;
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileTopY = cornerTileY * CMyTile::TileSize;
		if (cornerTileY == 0 || !CMyTile::IsTopOpen(cornerTileX, cornerTileY)) {
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
					tire.CollisionsCount += 1;
				} else {
					assert(false);
				}
			}
		}
	}

	// ������ ������.
	cornerIndex = (cornerIndex + 1) % 4;
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileRightX = (cornerTileX + 1) * CMyTile::TileSize;
		if (cornerTileX == CMyTile::SizeX() - 1 || !CMyTile::IsRightOpen(cornerTileX, cornerTileY)) {
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
					tire.CollisionsCount += 1;
				} else {
					assert(false);
				}
			}
		}
	}

	// ������ ������.
	cornerIndex = (cornerIndex + 1) % 4;
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileBottomY = (cornerTileY + 1) * CMyTile::TileSize;
		if (cornerTileY == CMyTile::SizeY() - 1 || !CMyTile::IsBottomOpen(cornerTileX, cornerTileY)) {
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
					tire.CollisionsCount += 1;
				} else {
					assert(false);
				}
			}
		}
	}

	// ��������� ����.
	const int tileX = static_cast<int>(tire.Position.X / 800);
	const int tileY = static_cast<int>(tire.Position.Y / 800);
	const double nearestTileCornerX = (tire.Position.X - tileX * 800 < 400) ? tileX * 800 : (tileX + 1) * 800;
	const double nearestTileCornerY = (tire.Position.Y - tileY * 800 < 400) ? tileY * 800 : (tileY + 1) * 800;
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
		tire.CollisionsCount += 1;
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

void CWorldSimulator::collideTireWithTire(CMyTire& tireA, CMyTire& tireB) const
{
	CCollisionInfo collisionInfo;
	CRotatedRect noRotatedRect1, noRotatedRect2;
	double collisionDeltaSpeed = 0;

	static const double collisionDistanceSqr = pow(CMyTire::Radius + CMyTire::Radius, 2);
	const double distanceSqr = (tireA.Position - tireB.Position).LengthSquared();
	if (distanceSqr < collisionDistanceSqr) {
		CVec2D vectorBA = CVec2D(tireB.Position, tireA.Position);
		collisionInfo.NormalB = vectorBA;
		collisionInfo.NormalB.Normalize();
		collisionInfo.Point = tireB.Position + vectorBA * (CMyTire::Radius / (CMyTire::Radius + CMyTire::Radius));
		collisionInfo.Depth = CMyTire::Radius + CMyTire::Radius - sqrt(distanceSqr);

		resolveCollision(collisionInfo,
			tireA.Position, tireA.Speed, tireA.AngularSpeed, noRotatedRect1,
			tireB.Position, tireB.Speed, tireB.AngularSpeed, noRotatedRect2, collisionDeltaSpeed,
			tireA.InvertedMass, tireA.InvertedAngularMass,
			tireB.InvertedMass, tireB.InvertedAngularMass,
			CMyTire::TireToTireMomentumTransferFactor, CMyTire::TireToTireSurfaceFrictionFactor);
		tireA.CollisionsCount += 1;
		tireB.CollisionsCount += 1;
	}
}

void CWorldSimulator::collideCarWithWalls(CMyCar& car) const
{
	double collisionDeltaSpeed = 0;
	CCollisionInfo collisionInfo;

	// ����� ����������� �� �������. ������ � ������ ������ ���� � ����� ������.
	double minLeft = INT_MAX;
	int cornerIndex = 0;
	const CVec2D* corners = car.RotatedRect.Corners;
	for (int i = 0; i < 4; i++) {
		if (corners[i].X < minLeft) {
			cornerIndex = i;
			minLeft = corners[i].X;
		}
	}
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileLeftX = cornerTileX * CMyTile::TileSize;
		if (cornerTileX == 0 || !CMyTile::IsLeftOpen(cornerTileX, cornerTileY)) {
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
	}

	// ������� ������.
	cornerIndex = (cornerIndex + 1) % 4;
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileTopY = cornerTileY * CMyTile::TileSize;
		if (cornerTileY == 0 || !CMyTile::IsTopOpen(cornerTileX, cornerTileY)) {
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
	}

	// ������ ������.
	cornerIndex = (cornerIndex + 1) % 4;
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileRightX = (cornerTileX + 1) * CMyTile::TileSize;
		if (cornerTileX == CMyTile::SizeX() - 1 || !CMyTile::IsRightOpen(cornerTileX, cornerTileY)) {
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
	}

	// ������ ������.
	cornerIndex = (cornerIndex + 1) % 4;
	{
		const int cornerTileX = static_cast<int>(corners[cornerIndex].X / 800);
		const int cornerTileY = static_cast<int>(corners[cornerIndex].Y / 800);
		const double tileBottomY = (cornerTileY + 1) * CMyTile::TileSize;
		if (cornerTileY == CMyTile::SizeY() - 1 || !CMyTile::IsBottomOpen(cornerTileX, cornerTileY)) {
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
	}

	// ��������� ����.
	const int tileX = static_cast<int>(car.Position.X / 800);
	const int tileY = static_cast<int>(car.Position.Y / 800);
	const double nearestTileCornerX = (car.Position.X - tileX * 800 < 400) ? tileX * 800 : (tileX + 1) * 800;
	const double nearestTileCornerY = (car.Position.Y - tileY * 800 < 400) ? tileY * 800 : (tileY + 1) * 800;
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

	// ���������� ������.
	static const double durabilityFactor = 0.003;
	static const double durabilityEps = 0.01;
	const double durabilityChange = durabilityFactor * collisionDeltaSpeed;
	if (collisionDeltaSpeed > 0) {
		car.CollisionDeltaSpeed += collisionDeltaSpeed;
		if(stopCollisions) car.CollisionsDetected += 1;
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
			// ���� ��������� ������� ������.
			continue;
		}
		for (int i = 0; i < 4; i++) {
			const CVec2D& p1 = car.RotatedRect.Corners[i];
			const CVec2D& p2 = car.RotatedRect.Corners[(i + 1) % 4];
			if (findLineWithCircleCollision(p1, p2, washer.Position, CMyWasher::Radius, collisionInfo)) {
				if (carId == washer.CarId) {
					// ��������, ��� ����� ������ ��� ���������� - ���� � ����� ��������� ������ ������,
					// ��� ���� ������ �������� ��������� �� ������.
					const CLine2D side = CLine2D::FromPoints(p1, p2);
					if (side.GetSignedDistanceFrom(washer.Position) < 0) {
						// ���� ��������� ������ ������ ������������ ������� ������������.
						continue;
					} else if (washer.Speed.DotProduct(collisionInfo.NormalB) < 0) {
						// ���� ��������� �������, �� � �������� ���������� �� �������, � ������� �������������� ������������.
						continue;
					}
				}
				resolveCollision(collisionInfo,
					car.Position, car.Speed, car.AngularSpeed, car.RotatedRect,
					washer.Position, washer.Speed, noAngularSpeed, noRotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(),
					washer.InvertedMass, washer.InvertedAngularMass,
					0.5, 0.25);
				// ���������� �����
				const double durabilityChange = min(0.15, car.Durability);
				if (durabilityChange > 0.01) {
					car.Durability = max(0.0, car.Durability - durabilityChange);
					const int washerPlayerId = world.Cars[washer.CarId].PlayerId;
					if (car.PlayerId != washerPlayerId) {
						world.Players[washerPlayerId].Score += static_cast<int>(100 * durabilityChange);
						world.Players[washerPlayerId].DamageScore += static_cast<int>(100 * durabilityChange);
						if (car.Durability == 0.0) {
							world.Players[washerPlayerId].Score += 100;
							world.Players[washerPlayerId].DamageScore += 100;;
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
	// ��������, �� ������� �����������.
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
			// ���� ��������� ������� ������.
			continue;
		}
		for (int i = 0; i < 4; i++) {
			const CVec2D& p1 = car.RotatedRect.Corners[i];
			const CVec2D& p2 = car.RotatedRect.Corners[(i + 1) % 4];
			if (findLineWithCircleCollision(p1, p2, tire.Position, CMyTire::Radius, collisionInfo)) {
				bool badSelfCollision = false;
				if (carId == tire.CarId) {
					if (tire.CollisionsCount == 0) {
						continue;
					}
					const CLine2D side = CLine2D::FromPoints(p1, p2);
					if (side.GetSignedDistanceFrom(tire.Position) < 0) {
						// ���� ��������� ������ ������ ������������ ������� ������������.
						collisionInfo.NormalB = -collisionInfo.NormalB;
						collisionInfo.Depth = CMyTire::Radius - collisionInfo.Depth;
						badSelfCollision = true;
					}
				}
				resolveCollision(collisionInfo,
					car.Position, car.Speed, car.AngularSpeed, car.RotatedRect,
					tire.Position, tire.Speed, tire.AngularSpeed, noRotatedRect, collisionDeltaSpeed,
					car.GetInvertedMass(), car.GetInvertedAngularMass(),
					tire.InvertedMass, tire.InvertedAngularMass,
					car.CarToTireMomentumTransferFactor, car.CarToTireSurfaceFrictionFactor);
				tire.CollisionsCount += 1;
				// ���������� �����
				const double durabilityChange = min(0.35 * collisionDeltaSpeed / 60.0, car.Durability);
				if (durabilityChange > 0.01) {
					car.Durability = max(0.0, car.Durability - durabilityChange);
					const int tirePlayerId = world.Cars[tire.CarId].PlayerId;
					if (car.PlayerId != tirePlayerId) {
						world.Players[tirePlayerId].Score += static_cast<int>(100 * durabilityChange);
						world.Players[tirePlayerId].DamageScore += static_cast<int>(100 * durabilityChange);
						if (car.Durability == 0.0) {
							world.Players[tirePlayerId].Score += 100;
							world.Players[tirePlayerId].DamageScore += 100;
						}
					}
				}
				if (badSelfCollision || tire.Speed.Length() < minTireSpeed) {
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
				car.MoneyCount += 1;
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

			// TODO: ��������� ����������� ��� ������� ������ �����
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

	if (stopCollisions) {
		speedA = { 0, 0 };
		angularSpeedA = 0;
	} else {
		CVec3D velocityChangeA = collisionNormalB * (impulseChange * invertedMassA);
		speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
		CVec3D angularVelocityChangeA = vectorAC.Cross(collisionNormalB * impulseChange) * invertedAngularMassA;
		angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;
	}
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

	if (stopCollisions) {
		speedA = { 0, 0 };
		angularSpeedA = 0;
	} else {
		CVec3D velocityChangeA = tangent * (impulseChange * invertedMassA);
		speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
		CVec3D angularVelocityChangeA = vectorAC.Cross(tangent * impulseChange) * invertedAngularMassA;
		angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;
	}
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


	if (stopCollisions) {
		speedA = { 0, 0 };
		angularSpeedA = 0;
		speedB = { 0, 0 };
		angularSpeedB = 0;
	} else {
		CVec3D velocityChangeA = collisionNormalB * (impulseChange * invertedMassA);
		speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
		CVec3D angularVelocityChangeA = vectorAC.Cross(collisionNormalB * impulseChange) * invertedAngularMassA;
		angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;

		CVec3D velocityChangeB = collisionNormalB * (impulseChange * invertedMassB);
		speedB = { speedB.X - velocityChangeB.X, speedB.Y - velocityChangeB.Y };
		CVec3D angularVelocityChangeB = vectorBC.Cross(collisionNormalB * impulseChange) * invertedAngularMassB;
		angularSpeedB = angularSpeedB - angularVelocityChangeB.Z;
	}
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

	if (stopCollisions) {
		speedA = { 0, 0 };
		angularSpeedA = 0;
		speedB = { 0, 0 };
		angularSpeedB = 0;
	} else {
		CVec3D velocityChangeA = tangent * (impulseChange * invertedMassA);
		speedA = { speedA.X + velocityChangeA.X, speedA.Y + velocityChangeA.Y };
		CVec3D angularVelocityChangeA = vectorAC.Cross(tangent * impulseChange) * invertedAngularMassA;
		angularSpeedA = angularSpeedA + angularVelocityChangeA.Z;

		CVec3D velocityChangeB = tangent * (impulseChange * invertedMassB);
		speedB = { speedB.X - velocityChangeB.X, speedB.Y - velocityChangeB.Y };
		CVec3D angularVelocityChangeB = vectorBC.Cross(tangent * impulseChange) * invertedAngularMassB;
		angularSpeedB = angularSpeedB - angularVelocityChangeB.Z;
	}
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
