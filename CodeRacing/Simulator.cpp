#include "Simulator.h"

#include <algorithm>
#include "math.h"
#include "assert.h"
#include "DrawPlugin.h"
#include "Log.h"
#include "MyTile.h"
#include "Tools.h"
#include "Vec3D.h"

using namespace std;

static const int subtickCount = 10;
static const double dTime = 1.0 / subtickCount;
static const double epsilon = 1e-7;

CSimulator::CSimulator() :
	isInitialized(false),
	forwardAccelByType(2, 0),
	rearAccelByType(2, 0),
	carLengthwiseFrictionFactorDt(0),
	carCrosswiseFrictionFactorDt(0),
	carRotationFrictionFactorDt(0),
	carMovementAirFrictionFactorDt(0),
	carRotationAirFrictionFactorDt(0)
{
}

void CSimulator::Initialize(const model::Game& _game)
{
	game = _game;

	static_assert(model::CarType::_CAR_TYPE_COUNT_ == 2, "CarType::_CAR_TYPE_COUNT_ assumed to be 2");
	forwardAccelByType[model::CarType::BUGGY] = game.getBuggyEngineForwardPower() / game.getBuggyMass();
	forwardAccelByType[model::CarType::JEEP] = game.getJeepEngineForwardPower() / game.getJeepMass();
	rearAccelByType[model::CarType::BUGGY] = game.getBuggyEngineRearPower() / game.getBuggyMass();
	rearAccelByType[model::CarType::JEEP] = game.getJeepEngineRearPower() / game.getJeepMass();

	carLengthwiseFrictionFactorDt = game.getCarLengthwiseMovementFrictionFactor() * dTime;
	carCrosswiseFrictionFactorDt = game.getCarCrosswiseMovementFrictionFactor() * dTime;
	carRotationFrictionFactorDt = game.getCarRotationFrictionFactor() * dTime;

	carMovementAirFrictionFactorDt = pow(1 - game.getCarMovementAirFrictionFactor(), dTime);
	carRotationAirFrictionFactorDt = pow(1 - game.getCarRotationAirFrictionFactor(), dTime);

	isInitialized = true;
}

bool CSimulator::IsInitialized() const
{
	return isInitialized;
}

static const double limit(double val, double lim)
{
	return max(-lim, min(lim, val));
}

CMyCar CSimulator::Predict(const CMyCar& startCar, const model::Move& move, int currentTick) const
{
	// TODO: Коллизии. Со стенами, мазутом, другими машинами, бонусами(!)
	// TODO: Правильно считать, если мы дохлые
	// TODO: Учитывать уже летящие в нас снаряды
	CMyCar car(startCar);
	car.CollisionDetected = false;

	bool isOiled = false;
	bool isBrake = false;
	CVec2D lengthwiseUnitVector;
	CVec2D accelerationDt;
	updateCar(move, currentTick, car, isOiled, isBrake, lengthwiseUnitVector, accelerationDt);

	CVec2D crosswiseUnitVector(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);
	updatePosition(car.Position, car.Speed, car.Angle, car.AngularSpeed, car.MedianAngularSpeed,
		lengthwiseUnitVector, crosswiseUnitVector,
		isBrake ? CVec2D(0, 0) : accelerationDt,
		carMovementAirFrictionFactorDt,
		isBrake && !isOiled ? carCrosswiseFrictionFactorDt : carLengthwiseFrictionFactorDt,
		isOiled ? carLengthwiseFrictionFactorDt : carCrosswiseFrictionFactorDt,
		carRotationAirFrictionFactorDt,
		isOiled ? carRotationFrictionFactorDt / 5 : carRotationFrictionFactorDt,
		false, -1, car.RotatedRect );

	return car;
}


CMyWasher CSimulator::Predict(const CMyWasher& startWasher, int /*currentTick*/) const
{
	CMyWasher washer(startWasher);
	double medianAngularSpeed = 0;
	double angularSpeed = 0;
	double angle = 0;
	CVec2D unit1(1, 0);
	CVec2D unit2(0, -1);
	CVec2D noAcc(0, 0);
	CRotatedRect noCorners;

	updatePosition(washer.Position, washer.Speed, angle, angularSpeed, medianAngularSpeed,
		unit1, unit2, noAcc,
		0, 0, 0, 0, 0,
		true, CMyWasher::Radius, noCorners);

	return washer;
}


CMyTire CSimulator::Predict(const CMyTire& startTire, int /*currentTick*/) const
{
	CMyTire tire(startTire);
	double medianAngularSpeed = 0;
	double angle = 0;
	CVec2D unit1(1, 0);
	CVec2D unit2(0, -1);
	CVec2D noAcc(0, 0);
	CRotatedRect noCorners;

	updatePosition(tire.Position, tire.Speed, angle, tire.AngularSpeed, medianAngularSpeed,
		unit1, unit2, noAcc,
		0, 0, 0, 0, 0,
		true, CMyWasher::Radius, noCorners);

	return tire;
}

void CSimulator::updateCar(const model::Move& move, int currentTick, CMyCar& car,
	bool& isOiled, bool& isBrake,
	CVec2D& lengthwiseUnitVector, CVec2D& accelerationDt) const
{
	lengthwiseUnitVector = { cos(car.Angle), sin(car.Angle) };

	// Проверка дохлости.
	if (car.Durability < 1e-5) {
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
	if (move.isUseNitro() && !isDead) {
		assert(car.NitroCount > 0 && car.NitroTicks == 0 && car.NitroCooldown == 0);
		car.NitroCount--;
		car.NitroTicks = game.getNitroDurationTicks();
		car.NitroCooldown = game.getUseNitroCooldownTicks();
	}
	const bool isNitro = car.NitroTicks > 0 && !isDead;
	car.NitroTicks = max(0, car.NitroTicks - 1);
	car.NitroCooldown = max(0, car.NitroCooldown - 1);

	// Лужа
	for (const auto& o : CGlobalPredictions::Oils) {
		if (currentTick >= o.LastTick || car.OiledTicks > 0) continue;
		// TODO: Уменьшать длительность лужи при наезде в неё.
		if ((car.Position - o.Position).LengthSquared() <= 150 * 150) {
			car.OiledTicks = min(o.LastTick - currentTick, game.getMaxOiledStateDurationTicks());
		}
	}
	isOiled = car.OiledTicks > 0;
	car.OiledTicks = max(0, car.OiledTicks - 1);

	// Тормоз
	isBrake = move.isBrake() && !isDead;

	// Команды на двигатель и на руль.
	const double enginePower = isDead ? 0 : move.getEnginePower();
	const double wheelTurn = isDead ? car.WheelTurn : move.getWheelTurn();

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
	accelerationDt = car.EnginePower >= 0 ?
		lengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
		lengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;

	// Обновляем угол поворота колёс и базовые (медианные) скорости.
	if (car.MedianAngularSpeed == UndefinedMedianAngularSpeed) {
		// Попытка оценить базовую скорость, если машина была создана из model::Car - там этих данных нет :(
		// К сожалению, такой хак приводит к небольшой потере точности при предсказании руления на первом тике.
		car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * lengthwiseUnitVector.DotProduct(car.Speed);
	}
	car.WheelTurn += limit(wheelTurn - car.WheelTurn, game.getCarWheelTurnChangePerTick());
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// Теперь считается базовая скорость на весь текущий тик.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * lengthwiseUnitVector.DotProduct(car.Speed);
	car.AngularSpeed += car.MedianAngularSpeed;
}

void CSimulator::updatePosition(
	CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed,
	CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
	double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
	double rotationAirFrictionFactorDt, double rotationFrictionFactorDt,
	bool passThroughWalls, double radius, CRotatedRect& rotatedRect ) const
{
	// Физика считается в несколько итераций.
	for (int i = 0; i < subtickCount; i++) {
		// Обновление позиции.
		position += speed * dTime;

		// Обновление скорости.
		// 1. Ускорение.
		speed += accelerationDt;
		// 2. Трение об воздух - пропорционально скорости и равномерно по всем направлениям.
		speed *= movementAirFrictionFactorDt;
		// 3. Трение колёс - постоянно и различно по направлениям. Если тормозим, то к продольному направлению надо
		//    применить такое же трение, что и к поперечному.
		const double frictionLengthwise = limit(speed.DotProduct(lengthwiseUnitVector), lengthwiseFrictionFactorDt);
		const double frictionCrosswise = limit(speed.DotProduct(crosswiseUnitVector), crosswiseFrictionFactorDt);
		speed -= lengthwiseUnitVector * frictionLengthwise + crosswiseUnitVector * frictionCrosswise;

		// Обновление угла.
		angle += angularSpeed * dTime;
		lengthwiseUnitVector = CVec2D(cos(angle), sin(angle));
		crosswiseUnitVector = CVec2D(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

		// Обновление угловой скорости.
		// Все трения применяются к той части, которая отличается от базовой скорости. Сначала воздушное трение, потом общее трение.
		angularSpeed -= medianAngularSpeed;
		angularSpeed *= rotationAirFrictionFactorDt;
		angularSpeed -= limit(angularSpeed, rotationFrictionFactorDt);
		angularSpeed += medianAngularSpeed;

		// Обработка коллизий.
		if (radius < 0) {
			rotatedRect = CRotatedRect(position, 210, 140, angle);
		}
		if (!passThroughWalls) {
			processWallsCollision(position, speed, angle, angularSpeed, radius, rotatedRect);
		}
	}

	normalizeAngle(angle);
}

void CSimulator::processWallsCollision(CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed,
	double radius, CRotatedRect& rotatedRect) const
{
	static const double tileSize = 800;
	static const double wallRadius = 80;
	if (radius > 0) {
		position;
		speed;
		angle;
		angularSpeed;
		radius;
		rotatedRect;

	} else {
		static const double momentumTransferFactor = 0.25;
		static const double surfaceFrictionFactor = 0.0625;

		// TODO откуда-то надо брать ширину и высоту, как и радиусы. а пока предполагаем, что
		// все прямоугольники, которые мы хотим обработать - это есть автомобиль.
		static const double width = 210;
		static const double height = 140;

		const double mass = 1; // todo: масса в зависимости от машины.
		const double invertedMass = 1 / mass;
		const double angularMass = 1.0 / 12 * mass * (width * width + height * height); // Момент инерции прямоугольника
		const double invertedAngularMass = 1 / angularMass;

		static const double biggerRadius = sqrt(pow(width / 2, 2) + pow(height / 2, 2));
		const int currentTileX = static_cast<int>(position.X / tileSize);
		const int currentTileY = static_cast<int>(position.Y / tileSize);
		const double currentCenterOffsetX = position.X - currentTileX * tileSize;
		const double currentCenterOffsetY = position.Y - currentTileY * tileSize;
		if (tileSize + biggerRadius < currentCenterOffsetX && currentCenterOffsetX < tileSize - wallRadius - biggerRadius &&
			tileSize + biggerRadius < currentCenterOffsetY && currentCenterOffsetY < tileSize - wallRadius - biggerRadius)
		{
			return; //nocollision
		}
		const int minTileX = static_cast<int>((position.X - biggerRadius) / tileSize);
		const int maxTileX = static_cast<int>((position.X + biggerRadius) / tileSize);
		const int minTileY = static_cast<int>((position.Y - biggerRadius) / tileSize);
		const int maxTileY = static_cast<int>((position.Y + biggerRadius) / tileSize);

		for (int tileX = minTileX; tileX <= maxTileX; tileX++) {
			for (int tileY = minTileY; tileY <= maxTileY; tileY++) {
				// TODO: Внутренние углы пока считать не умеем :(
				CMyTile tile(tileX, tileY);
				auto straightWalls = tile.GetStraightWalls();
				for (const auto& w : straightWalls) 
				{
					CVec2D collisionNormalB;
					CVec2D collisionPoint;
					double depth = 0;
					if (findLineWithRotatedRectCollision(w.first, w.second,
						position, rotatedRect, biggerRadius,
						collisionNormalB, collisionPoint, depth))
					{
						resolveCollisionStatic(collisionNormalB, collisionPoint, depth,
							position, speed, angularSpeed, rotatedRect,
							invertedMass, invertedAngularMass, momentumTransferFactor, surfaceFrictionFactor);
					}
				}
			}
		}
	}
}

bool CSimulator::findLineWithRotatedRectCollision(
	const CVec2D& point1A, const CVec2D& point2A,
	const CVec2D& position, const CRotatedRect& rotatedRect, double circumcircleRadius,
	CVec2D& collisionNormalB, CVec2D& collisionPoint, double& depth) const
{
	CLine2D lineA = CLine2D::FromPoints(point1A, point2A);
	if (abs(lineA.GetSignedDistanceFrom(position)) > circumcircleRadius) {
		return false;
	}

	CLine2D intersectionLineB;
	static const int maxIntersectionPoints = 2;
	CVec2D intersectionPoints[maxIntersectionPoints];
	int intersectionPointsCount = 0;

	static const int pointBCount = 4;
	for (int pointBIndex = 0; pointBIndex < pointBCount; pointBIndex++) {
		const CVec2D& point1B = rotatedRect.Corners[pointBIndex];
		const CVec2D& point2B = rotatedRect.Corners[pointBIndex == pointBCount - 1 ? 0 : pointBIndex + 1];
		CLine2D lineB = CLine2D::FromPoints(point1B, point2B);
		CVec2D intersectionPoint;
		if (!lineA.GetIntersectionPoint(lineB, intersectionPoint)) {
			continue;
		}
		const double left = max(min(point1A.X, point2A.X), min(point1B.X, point2B.X));
		const double top = max(min(point1A.Y, point2A.Y), min(point1B.Y, point2B.Y));
		const double right = min(max(point1A.X, point2A.X), max(point1B.X, point2B.X));
		const double bottom = min(max(point1A.Y, point2A.Y), max(point1B.Y, point2B.Y));
		if (intersectionPoint.X <= left - epsilon
			|| intersectionPoint.X >= right + epsilon
			|| intersectionPoint.Y <= top - epsilon
			|| intersectionPoint.Y >= bottom + epsilon)
		{
			continue;
		}
		intersectionLineB = lineB;
		assert(intersectionPointsCount < maxIntersectionPoints);
		if (intersectionPointsCount > 0 && intersectionPoint.NearlyEquals(intersectionPoints[0])) {
			continue;
		}
		intersectionPoints[intersectionPointsCount] = intersectionPoint;
		intersectionPointsCount++;
	}

	if (intersectionPointsCount == 0) {
		return false; // TODO check line inside rectangle
	} else if (intersectionPointsCount == 1) {
		collisionNormalB = intersectionLineB.GetProjectionOf(position) - position;
		collisionNormalB *= 1 / collisionNormalB.Length();
		CLine2D parallelLine1A = intersectionLineB.GetParallelLine(point1A);
		double distance1AFromB = parallelLine1A.GetDistanceFrom(position);
		CLine2D parallelLine2A = intersectionLineB.GetParallelLine(point2A);
		double distance2AFromB = parallelLine2A.GetDistanceFrom(position);
		depth = (distance1AFromB < distance2AFromB ? parallelLine1A : parallelLine2A).GetDistanceFrom(intersectionLineB);
		collisionPoint = intersectionPoints[0];
		return true;
	} else {
		assert(intersectionPointsCount == 2);
		CVec2D pointBWithMinDistanceFromA = rotatedRect.Corners[0];
		double minDistanceBFromA = lineA.GetSignedDistanceFrom(pointBWithMinDistanceFromA);
		CVec2D pointBWithMaxDistanceFromA = pointBWithMinDistanceFromA;
		double maxDistanceBFromA = minDistanceBFromA;
		for (const auto& p : rotatedRect.Corners) {
			double distanceBFromA = lineA.GetSignedDistanceFrom(p);
			if (distanceBFromA < minDistanceBFromA) {
				minDistanceBFromA = distanceBFromA;
				pointBWithMinDistanceFromA = p;
			}
			if (distanceBFromA > maxDistanceBFromA) {
				maxDistanceBFromA = distanceBFromA;
				pointBWithMaxDistanceFromA = p;
			}
		}

		if (minDistanceBFromA < 0 && maxDistanceBFromA < 0 || minDistanceBFromA > 0 && maxDistanceBFromA > 0) {
			return false;
		}

		if (lineA.GetSignedDistanceFrom(position) > 0) {
			// пипец какой-то со знаками
			//collisionNormalB = lineA.GetParallelLine(pointBWithMinDistanceFromA).GetUnitNormalFrom(pointBWithMaxDistanceFromA);
			collisionNormalB = -lineA.GetParallelLine(pointBWithMinDistanceFromA).GetUnitNormalFrom(pointBWithMaxDistanceFromA);
			depth = abs(minDistanceBFromA);
		} else {
			collisionNormalB = lineA.GetParallelLine(pointBWithMaxDistanceFromA).GetUnitNormalFrom(pointBWithMinDistanceFromA);
			depth = maxDistanceBFromA;
		}

		double averageIntersectionX = 0;
		double averageIntersectionY = 0;
		for (int i = 0; i < intersectionPointsCount; i++) {
			averageIntersectionX += intersectionPoints[i].X / intersectionPointsCount;
			averageIntersectionY += intersectionPoints[i].Y / intersectionPointsCount;
		}

		collisionPoint = CVec2D(averageIntersectionX, averageIntersectionY);
		return true;
	}
}

void CSimulator::resolveCollisionStatic(
	const CVec2D& collisionNormalB2D, const CVec2D& collisionPoint, double depth,
	CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRect,
	double invertedMassA, double invertedAngularMassA,
	double momentumTransferFactorAB, double surfaceFrictionFactorAB) const
{
	CVec3D collisionNormalB(collisionNormalB2D);
	CVec3D vectorAC(collisionPoint - positionA);
	CVec3D angularVelocityPartAC = CVec3D(angularSpeedA).Cross(vectorAC);
	CVec3D velocityAC = angularVelocityPartAC + CVec3D(speedA);
	CVec3D relativeVelocityC = velocityAC;

	const double normalRelativeVelocityLengthC = -relativeVelocityC.DotProduct(collisionNormalB);
	CLog::Instance().Stream() << "Impact velocity: " << normalRelativeVelocityLengthC << endl;
	if (normalRelativeVelocityLengthC > -epsilon) {
		resolveImpactStatic(vectorAC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA,
			invertedMassA, invertedAngularMassA, momentumTransferFactorAB);
		resolveSurfaceFrictionStatic(vectorAC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA,
			invertedMassA, invertedAngularMassA, surfaceFrictionFactorAB);
	}
	pushBackBodiesStatic(collisionNormalB2D, depth, positionA, rotatedRect);
	// TODO: обновить прочность. dDurability = 0.003 * |relativeVelocityC|, порог 0.01
}

void CSimulator::resolveImpactStatic(
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

void CSimulator::resolveSurfaceFrictionStatic(
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

void CSimulator::pushBackBodiesStatic(
	const CVec2D& collisionNormalB2D, double depth,
	CVec2D& positionA, CRotatedRect& rotatedRect) const
{
	const CVec2D shift = collisionNormalB2D * (depth + epsilon);
	positionA += shift;
	for (auto& c : rotatedRect.Corners) {
		c += shift;
	}
}
