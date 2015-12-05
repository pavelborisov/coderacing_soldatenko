#include "Simulator.h"

#ifdef LOGGING
#undef NDEBUG
#endif

#include <algorithm>
#include "math.h"
#include "assert.h"
#include "DrawPlugin.h"
#include "Log.h"
#include "MyTile.h"
#include "Tools.h"
#include "Vec3D.h"

using namespace std;

static const int subtickCount = 2;
static const double dTime = 1.0 / subtickCount;
static const double epsilon = 1e-7;

static const double tileSize = 800;
static const double wallRadius = 80;
static const double wallOffsetMin = wallRadius;
static const double wallOffsetMin2 = wallRadius * 2;
static const double wallOffsetMax = tileSize - wallRadius;
static const double wallOffsetMax2 = tileSize - 2 * wallRadius;
static const double carWidth = 210;
static const double carHeight = 140;
static const double carCircumCircleRadius = sqrt(carWidth * carWidth + carHeight * carHeight) / 2;

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
	// TODO: Идея - считать отскоки коллизий только на начальных тиках.
	// TODO: Коллизии. Со стенами, мазутом, другими машинами, бонусами(!)
	// TODO: Правильно считать, если мы дохлые
	// TODO: Учитывать уже летящие в нас снаряды
	CMyCar car(startCar);
	car.CollisionDeltaSpeed = 0;

	bool isOiled = false;
	bool isBrake = false;
	CVec2D lengthwiseUnitVector;
	CVec2D accelerationDt;
	updateCar(move, currentTick, car, isOiled, isBrake, lengthwiseUnitVector, accelerationDt);

	CVec2D crosswiseUnitVector(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);
	updatePosition(car.Position, car.Speed, car.Angle, car.AngularSpeed, car.MedianAngularSpeed, car.RotatedRect, car.CollisionDeltaSpeed,
		lengthwiseUnitVector, crosswiseUnitVector,
		isBrake ? CVec2D(0, 0) : accelerationDt,
		carMovementAirFrictionFactorDt,
		isBrake && !isOiled ? carCrosswiseFrictionFactorDt : carLengthwiseFrictionFactorDt,
		isOiled ? carLengthwiseFrictionFactorDt : carCrosswiseFrictionFactorDt,
		carRotationAirFrictionFactorDt,
		isOiled ? carRotationFrictionFactorDt / 5 : carRotationFrictionFactorDt,
		false, -1);

	static const double durabilityFactor = 0.003;
	static const double durabilityEps = 0.01;
	const double durabilityChange = durabilityFactor * car.CollisionDeltaSpeed;
	if (car.CollisionDeltaSpeed > 0) {
		car.CollisionsDetected += 1;
	}
	if (durabilityChange >= durabilityEps) {
		car.Durability = max(0.0, car.Durability - durabilityChange);
	}

	if (car.Durability < startCar.Durability) {
		//CLog::Instance().Stream() << "There was a collision for " << startCar.Durability - car.Durability << " durability damage" << endl;
		//car.CollisionDetected = true;
	}
	for (const auto& c : car.RotatedRect.Corners) {
		if (c.X > CMyTile::SizeX() * 800 - 80) {
			assert(false);
		}
		if (c.X < 80) {
			assert(false);
		}
		if (c.Y > CMyTile::SizeY() * 800 - 80) {
			assert(false);
		}
		if (c.Y < 80) {
			assert(false);
		}
	}

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
	double collisionDeltaSpeed = 0;

	updatePosition(washer.Position, washer.Speed, angle, angularSpeed, medianAngularSpeed, noCorners, collisionDeltaSpeed,
		unit1, unit2, noAcc,
		0, 0, 0, 0, 0,
		true, CMyWasher::Radius);

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
	double collisionDeltaSpeed = 0;

	updatePosition(tire.Position, tire.Speed, angle, tire.AngularSpeed, medianAngularSpeed, noCorners, collisionDeltaSpeed,
		unit1, unit2, noAcc,
		0, 0, 0, 0, 0,
		true, CMyWasher::Radius);

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
	CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed, CRotatedRect& rotatedRect, double& collisionDeltaSpeed,
	CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
	double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
	double rotationAirFrictionFactorDt, double rotationFrictionFactorDt,
	bool passThroughWalls, double radius) const
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
			rotatedRect = CRotatedRect(position, carWidth, carHeight, angle);
		}
		if (!passThroughWalls) {
			//processWallsCollision(position, speed, angle, angularSpeed, radius, rotatedRect, collisionDeltaSpeed);
			processCarWithWallsCollision(position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed);
		}
	}

	normalizeAngle(angle);
}

void CSimulator::processCarWithWallsCollision(CVec2D& position, CVec2D& speed, double& angularSpeed,
	CRotatedRect& rotatedRect, double& collisionDeltaSpeed) const
{
	// TODO: Сделать все массы зависимые от машины.
	static const double carMass = 1;
	static const double carInvertedMass = 1 / carMass;
	static const double carAngularMass = 1.0 / 12 * carMass * (carWidth * carWidth + carHeight * carHeight); // Момент инерции прямоугольника
	static const double carInvertedAngularMass = 1 / carAngularMass;
	static const double carToWallMomentumTransferFactor = 0.25;
	static const double carToWallSurfaceFrictionFactor = 0.25 * 0.25; // 0.0625

	// Эта операция вызывается очень много раз. Так что отсечка по тому, было ли столкновение должно работать очень быстро.
	// Определим столкновение по восьми точкам, раскиданным по машине - в углах и в центрах сторон.
	CCollisionInfo collisionInfo;
	for (int i = 0; i < 4; i++ ) {
		for (int j = 0; j < 2; j++) {
			CVec2D p = j == 0 ? rotatedRect.Corners[i] : (rotatedRect.Corners[i] + rotatedRect.Corners[(i + 1) % 4]) * 0.5;
			const CMyTile tile(static_cast<int>(p.X / tileSize), static_cast<int>(p.Y / tileSize));
			const double tileX = tile.X * tileSize;
			const double tileY = tile.Y * tileSize;
			const double offsetX = p.X - tileX;
			const double offsetY = p.Y - tileY;
			const bool leftOpen = tile.IsLeftOpen();
			const bool rightOpen = tile.IsRightOpen();
			const bool topOpen = tile.IsTopOpen();
			const bool bottomOpen = tile.IsBottomOpen();
			// Для каждой стены будем смотреть её прямую стену и один из углов.

			// Левая стена и левый верхний угол.
			if (!leftOpen) {
				if (!topOpen && offsetX < wallOffsetMin2 && offsetY < wallOffsetMin2) {
					// Вогнутая арка.
					CArc2D arc(CVec2D(tileX + wallOffsetMin2, tileY + wallOffsetMin2), wallRadius, -PI, -PI / 2);
					if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
				if (offsetX < wallOffsetMin) {
					// Прямая стена.
					//CVec2D start = { tileX + wallOffsetMin, tileY }, end = { tileX + wallOffsetMin, tileY + tileSize };
					CVec2D start = { tileX + wallOffsetMin, tileY - tileSize }, end = { tileX + wallOffsetMin, tileY + 2 * tileSize };
					if (findLineWithRotatedRectCollision(start, end, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
			} else if (topOpen && offsetX < wallOffsetMin && offsetY < wallOffsetMin) {
				// Выпуклая арка
				//CArc2D arc(CVec2D(tileX, tileY), wallRadius, 0, PI / 2);
				CArc2D arc(CVec2D(tileX, tileY), wallRadius, -PI, PI);
				if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
					resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
						carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
				}
			}

			// Верхняя стена и верхний правый угол.
			if (!topOpen) {
				if (!rightOpen && offsetX > wallOffsetMax2 && offsetY < wallOffsetMin2) {
					// Вогнутая арка.
					CArc2D arc(CVec2D(tileX + wallOffsetMax2, tileY + wallOffsetMin2), wallRadius, -PI / 2, 0);
					if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
				if (offsetY < wallOffsetMin) {
					// Прямая стена.
					//CVec2D start = { tileX + tileSize, tileY + wallOffsetMin }, end = { tileX, tileY + wallOffsetMin };
					CVec2D start = { tileX + 2 * tileSize, tileY + wallOffsetMin }, end = { tileX - tileSize, tileY + wallOffsetMin };
					if (findLineWithRotatedRectCollision(start, end, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
			} else if (rightOpen && offsetX > wallOffsetMax && offsetY < wallOffsetMin) {
				// Выпуклая арка.
				//CArc2D arc(CVec2D(tileX + tileSize, tileY), wallRadius, PI / 2, PI);
				CArc2D arc(CVec2D(tileX + tileSize, tileY), wallRadius, -PI, PI);
				if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
					resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
						carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
				}
			}

			// Правая стена и правый нижний угол.
			if (!rightOpen) {
				if (!bottomOpen && offsetX > wallOffsetMax2 && offsetY > wallOffsetMax2) {
					CArc2D arc(CVec2D(tileX + wallOffsetMax2, tileY + wallOffsetMax2), wallRadius, 0, PI / 2);
					if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
				if (offsetX > wallOffsetMax) {
					//CVec2D start = { tileX + wallOffsetMax, tileY + tileSize }, end = { tileX + wallOffsetMax, tileY };
					CVec2D start = { tileX + wallOffsetMax, tileY + 2 * tileSize }, end = { tileX + wallOffsetMax, tileY - tileSize };
					if (findLineWithRotatedRectCollision(start, end, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
			} else if (bottomOpen && offsetX > wallOffsetMax && offsetY > wallOffsetMax) {
				//CArc2D arc(CVec2D(tileX + tileSize, tileY + tileSize), wallRadius, -PI, -PI / 2);
				CArc2D arc(CVec2D(tileX + tileSize, tileY + tileSize), wallRadius, -PI, PI);
				if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
					resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
						carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
				}
			}

			// Нижняя стена и нижний левый угол.
			if (!bottomOpen) {
				if (!leftOpen && offsetX < wallOffsetMin2 && offsetY > wallOffsetMax2) {
					CArc2D arc(CVec2D(tileX + wallOffsetMin2, tileY + wallOffsetMax2), wallRadius, PI / 2, PI);
					if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
				if (offsetY > wallOffsetMax) {
					//CVec2D start = { tileX, tileY + wallOffsetMax }, end = { tileX + tileSize, tileY + wallOffsetMax };
					CVec2D start = { tileX - tileSize, tileY + wallOffsetMax }, end = { tileX + 2 * tileSize, tileY + wallOffsetMax };
					if (findLineWithRotatedRectCollision(start, end, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
						resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
							carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
					}
				}
			} else if (leftOpen && offsetX < wallOffsetMin && offsetY > wallOffsetMax) {
				//CArc2D arc(CVec2D(tileX, tileY + tileSize), wallRadius, -PI / 2, PI);
				CArc2D arc(CVec2D(tileX, tileY + tileSize), wallRadius, -PI, PI);
				if (findArcWithRotatedRectCollision(arc, position, rotatedRect, carCircumCircleRadius, collisionInfo)) {
					resolveCollisionStatic(collisionInfo, position, speed, angularSpeed, rotatedRect, collisionDeltaSpeed,
						carInvertedMass, carInvertedAngularMass, carToWallMomentumTransferFactor, carToWallSurfaceFrictionFactor);
				}
			}
		}
	}
}

bool CSimulator::findLineWithRotatedRectCollision(
	const CVec2D& point1A, const CVec2D& point2A,
	const CVec2D& positionB, const CRotatedRect& rotatedRectB, double circumcircleRadiusB,
	CCollisionInfo& collisionInfo) const
{
	// Надо обратить внимание, что тут тело "B" это наш прямоугольник, а "A" - линия.
	CLine2D lineA = CLine2D::FromPoints(point1A, point2A);
	if (abs(lineA.GetSignedDistanceFrom(positionB)) > circumcircleRadiusB) {
		return false;
	}

	CLine2D intersectionLineB;
	static const int maxIntersectionPoints = 3;
	CVec2D intersectionPoints[maxIntersectionPoints];
	int intersectionPointsCount = 0;

	static const int pointBCount = 4;
	for (int pointBIndex = 0; pointBIndex < pointBCount; pointBIndex++) {
		const CVec2D& point1B = rotatedRectB.Corners[pointBIndex];
		const CVec2D& point2B = rotatedRectB.Corners[pointBIndex == pointBCount - 1 ? 0 : pointBIndex + 1];
		CLine2D lineB = CLine2D::FromPoints(point1B, point2B);

		if (lineB.GetSignedDistanceFrom(positionB) > 0) {
			assert(false);
		}

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
		intersectionPoints[intersectionPointsCount] = intersectionPoint;
		intersectionPointsCount++;
	}

	if (intersectionPointsCount == 0) {
		return false; // TODO check line inside rectangle
	} else if (intersectionPointsCount == 1) {
		// TODO: Есть баг, когда точка машины находится от стенки на расстоянии меньше epsilon, 
		// и на предыдущем этапе одно продолжение грани машины считается, что пересеклось со стеной,
		// а другое - уже нет. Это зависит от угла машины. В общем, может так получиться, что вместо 0 или 2 точек
		// пересечения появляется одна. И этот код нас выносит за пределы стены далеко-далеко.
		//return false;
		// Проверка на то, что хотя бы один край стены находится внутри прямоугольника
		bool isPoint1AInside = true;
		bool isPoint2AInside = true;
		for (int pointBIndex = 0; pointBIndex < pointBCount; pointBIndex++) {
			const CVec2D& point1B = rotatedRectB.Corners[pointBIndex];
			const CVec2D& point2B = rotatedRectB.Corners[pointBIndex == pointBCount - 1 ? 0 : pointBIndex + 1];
			CLine2D line = CLine2D::FromPoints(point1B, point2B);
			if (line.GetSignedDistanceFrom(point1A) > 0) {
				isPoint1AInside = false;
			}
			if (line.GetSignedDistanceFrom(point2A) > 0) {
				isPoint2AInside = false;
			}
		}
		if (!isPoint1AInside && !isPoint2AInside) {
			return false;
		}
		
		collisionInfo.Normal = intersectionLineB.GetProjectionOf(positionB) - positionB;
		collisionInfo.Normal *= 1 / collisionInfo.Normal.Length();
		collisionInfo.Normal = -collisionInfo.Normal;
		CLine2D parallelLine1A = intersectionLineB.GetParallelLine(point1A);
		double distance1AFromB = parallelLine1A.GetDistanceFrom(positionB);
		CLine2D parallelLine2A = intersectionLineB.GetParallelLine(point2A);
		double distance2AFromB = parallelLine2A.GetDistanceFrom(positionB);
		collisionInfo.Depth = (distance1AFromB < distance2AFromB ? parallelLine1A : parallelLine2A).GetDistanceFrom(intersectionLineB);
		collisionInfo.Point = intersectionPoints[0];
		return true;
	} else {
		//assert(intersectionPointsCount == 2);
		CVec2D pointBWithMinDistanceFromA = rotatedRectB.Corners[0];
		double minDistanceBFromA = lineA.GetSignedDistanceFrom(pointBWithMinDistanceFromA);
		CVec2D pointBWithMaxDistanceFromA = pointBWithMinDistanceFromA;
		double maxDistanceBFromA = minDistanceBFromA;
		for (const auto& p : rotatedRectB.Corners) {
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

		if (lineA.GetSignedDistanceFrom(positionB) > 0) {
			//collisionInfo.Normal = lineA.GetParallelLine(pointBWithMinDistanceFromA).GetUnitNormalFrom(pointBWithMaxDistanceFromA);
			collisionInfo.Normal = -lineA.GetParallelLine(pointBWithMinDistanceFromA).GetUnitNormalFrom(pointBWithMaxDistanceFromA);
			collisionInfo.Depth = abs(minDistanceBFromA);
		} else {
			//collisionInfo.Normal = lineA.GetParallelLine(pointBWithMaxDistanceFromA).GetUnitNormalFrom(pointBWithMinDistanceFromA);
			collisionInfo.Normal = -lineA.GetParallelLine(pointBWithMaxDistanceFromA).GetUnitNormalFrom(pointBWithMinDistanceFromA);
			collisionInfo.Depth = maxDistanceBFromA;
		}

		double averageIntersectionX = 0;
		double averageIntersectionY = 0;
		for (int i = 0; i < intersectionPointsCount; i++) {
			averageIntersectionX += intersectionPoints[i].X / intersectionPointsCount;
			averageIntersectionY += intersectionPoints[i].Y / intersectionPointsCount;
		}

		collisionInfo.Point = CVec2D(averageIntersectionX, averageIntersectionY);
		return true;
	}
}

static void updateNearestPoint(
	const CVec2D& positionB, const CVec2D& point, CVec2D& nearestPoint, double& distanceToNearestPoint)
{
	double distanceToPoint = (positionB - point).Length();

	if (distanceToPoint >= epsilon
		&& (distanceToNearestPoint == INT_MAX || distanceToPoint < distanceToNearestPoint))
	{
		nearestPoint = point;
		distanceToNearestPoint = distanceToPoint;
	}
}

static void updateFarthestPoint(
	const CVec2D& positionB, const CVec2D& point, CVec2D& farthestPoint, double& distanceToFarthestPoint,
	double startAngle, double finishAngle)
{
	double distanceToPoint = (positionB - point).Length();

	//if (GeometryUtil.isAngleBetween(new Vector2D(body.getPosition(), point).getAngle(), startAngle, finishAngle)
	CVec2D angleVector(positionB, point);
	const double angle = angleVector.GetAngle();
	if (startAngle <= angle && angle <= finishAngle
		&& (distanceToFarthestPoint == INT_MIN || distanceToPoint > distanceToFarthestPoint))
	{
		farthestPoint = point;
		distanceToFarthestPoint = distanceToPoint;
	}
}

static bool doesPointBelongToAAndB(
	const CVec2D& point, double leftA, double topA, double rightA, double bottomA,
	const CArc2D& arcB)
{
	bool belongsToA = (point.X > leftA - epsilon)
		&& (point.X < rightA + epsilon)
		&& (point.Y > topA - epsilon)
		&& (point.Y < bottomA + epsilon);

	double pointAngleB = CVec2D(arcB.Center, point).GetAngle();
	//if (pointAngleB < startAngleB) {
	//	pointAngleB += DOUBLE_PI;
	//}

	bool belongsToB = arcB.StartAngle <= pointAngleB && pointAngleB <= arcB.FinishAngle;

	return belongsToA && belongsToB;
}

struct CIntersectionInfo {
	CVec2D Point;
	vector<CLine2D> Lines;
	vector<pair<CVec2D, CVec2D>> LinePointPairs;
};

static void addIntersectionInfo(
	const CVec2D& point, const CVec2D& point1A, const CVec2D& point2A, const CLine2D& lineA, vector<CIntersectionInfo>& intersectionInfos)
{
	bool alreadyAdded = false;

	for (auto& intersectionInfo : intersectionInfos) {
		if (intersectionInfo.Point.NearlyEquals(point)) {
			intersectionInfo.Lines.push_back(lineA);
			intersectionInfo.LinePointPairs.push_back(make_pair(point1A, point2A));
			alreadyAdded = true;
			break;
		}
	}

	if (!alreadyAdded) {
		intersectionInfos.push_back(CIntersectionInfo());
		CIntersectionInfo& intersectionInfo = intersectionInfos.back();
		intersectionInfo.Point = point;
		intersectionInfo.Lines.push_back(lineA);
		intersectionInfo.LinePointPairs.push_back(make_pair(point1A, point2A));
	}
}

bool CSimulator::findArcWithRotatedRectCollision(
	const CArc2D& arcB,
	const CVec2D& positionA, const CRotatedRect& rotatedRectA, double circumcircleRadiusA,
	CCollisionInfo& collisionInfo) const
{
	// Упрощённый случай - мы знаем, что у нас есть только два случая: 
	// 1) когда мы углом машины попали в "вогнутую" арку - тогда центр окружности находится внутри автомобиля и есть 2 точки пересечения.
	// 2) когда мы углом или стороной машины попали в "выпуклую" арку - тогда центр окружности находится снаружи автомобиля и есть 2 точки пересечения.
	const double radiusA = circumcircleRadiusA;
	const double radiusB = arcB.Radius;
	const double distanceSqr = (arcB.Center - positionA).LengthSquared();
	if (distanceSqr > pow(radiusA + radiusB, 2)) {
		return false;
	}
	const double radiusBSqr = pow(arcB.Radius, 2);

	//static const int maxIntersectionPoints = 2;
	//CVec2D intersectionPoints[maxIntersectionPoints];
	//int intersectionPointsCount = 0;
	//static const int maxIntersectionInfos = 4;
	vector<CIntersectionInfo> intersectionInfos;

	static const int pointACount = 4;
	for (int pointAIndex = 0; pointAIndex < pointACount; pointAIndex++) {
		const CVec2D& point1A = rotatedRectA.Corners[pointAIndex];
		const CVec2D& point2A = rotatedRectA.Corners[pointAIndex == pointACount - 1 ? 0 : pointAIndex + 1];
		CLine2D lineA = CLine2D::FromPoints(point1A, point2A);

		if (lineA.GetSignedDistanceFrom(positionA) > 0) {
			assert(false);
		}

		const double distanceFromB = lineA.GetSignedDistanceFrom(arcB.Center);
		if (distanceFromB > arcB.Radius) {
			continue;
		}
		// Точки пересечения окружности с прямой находятся в projectionOfB +- offsetVector
		CVec2D projectionOfB = lineA.GetProjectionOf(arcB.Center);
		const double offset = sqrt(radiusBSqr - pow(distanceFromB, 2));
		CVec2D offsetVector = CVec2D(point1A, point2A);
		offsetVector *= offset / offsetVector.Length();

		const double leftA = min(point1A.X, point2A.X);
		const double topA = min(point1A.Y, point2A.Y);
		const double rightA = max(point1A.X, point2A.X);
		const double bottomA = max(point1A.Y, point2A.Y);

		CVec2D intersectionPoint1 = projectionOfB + offsetVector;
		if (doesPointBelongToAAndB(intersectionPoint1, leftA, topA, rightA, bottomA, arcB)) {
			addIntersectionInfo(intersectionPoint1, point1A, point2A, lineA, intersectionInfos);
		}
		//assert(intersectionInfos.size() < maxIntersectionInfos);
		
		CVec2D intersectionPoint2 = projectionOfB - offsetVector;
		if (doesPointBelongToAAndB(intersectionPoint1, leftA, topA, rightA, bottomA, arcB)) {
			addIntersectionInfo(intersectionPoint2, point1A, point2A, lineA, intersectionInfos);
		}
		//assert(intersectionInfos.size() < maxIntersectionInfos);
	}

	const int intersectionCount = intersectionInfos.size();
	if (intersectionCount == 0) {
		// TODO check arc inside rectangle
		return false;
	} else if (intersectionCount == 1) {
		// Пока такое не считаем
		return false;
	} else {
		//assert(intersectionCount == 2);
		CVec2D vectorCB = arcB.Center - intersectionInfos[0].Point;
		CVec2D vectorCA = positionA - intersectionInfos[0].Point;
		const double distance = sqrt(distanceSqr);
		if (distance > radiusB - epsilon && vectorCB.DotProduct(vectorCA) < 0) {
			CVec2D nearestPoint;
			double distanceToNearestPoint = INT_MAX;
			for (const auto& ii : intersectionInfos) {
				updateNearestPoint(arcB.Center, ii.Point, nearestPoint, distanceToNearestPoint);
				for (const auto& pp : ii.LinePointPairs) {
					updateNearestPoint(arcB.Center, pp.first, nearestPoint, distanceToNearestPoint);
					updateNearestPoint(arcB.Center, pp.second, nearestPoint, distanceToNearestPoint);
				}
			}
			if (distanceToNearestPoint == INT_MAX) {
				return false;
			}
			collisionInfo.Normal = nearestPoint - arcB.Center;
			collisionInfo.Normal *= 1 / collisionInfo.Normal.Length();
			collisionInfo.Point = nearestPoint;
			collisionInfo.Depth = radiusB - distanceToNearestPoint;
			return true;
		} else {
			CVec2D farthestPoint;
			double distanceToFarthestPoint = INT_MIN;
			for (const auto& ii : intersectionInfos) {
				updateFarthestPoint(arcB.Center, ii.Point, farthestPoint, distanceToFarthestPoint, arcB.StartAngle, arcB.FinishAngle);
				for (const auto& pp : ii.LinePointPairs) {
					updateFarthestPoint(arcB.Center, pp.first, farthestPoint, distanceToFarthestPoint, arcB.StartAngle, arcB.FinishAngle);
					updateFarthestPoint(arcB.Center, pp.second, farthestPoint, distanceToFarthestPoint, arcB.StartAngle, arcB.FinishAngle);
				}
			}
			if (distanceToFarthestPoint == INT_MIN) {
				return false;
			}
			collisionInfo.Normal = arcB.Center - farthestPoint;
			collisionInfo.Normal *= 1 / collisionInfo.Normal.Length();
			collisionInfo.Point = farthestPoint;
			collisionInfo.Depth = distanceToFarthestPoint - radiusB;
			return true;
		}
	}

	return false;
}

void CSimulator::resolveCollisionStatic(
	const CCollisionInfo& collisionInfo,
	CVec2D& positionA, CVec2D& speedA, double& angularSpeedA, CRotatedRect& rotatedRect, double& collisionDeltaSpeed,
	double invertedMassA, double invertedAngularMassA,
	double momentumTransferFactorAB, double surfaceFrictionFactorAB) const
{
	CVec3D collisionNormalB(collisionInfo.Normal);
	CVec3D vectorAC(collisionInfo.Point - positionA);
	CVec3D angularVelocityPartAC = CVec3D(angularSpeedA).Cross(vectorAC);
	CVec3D velocityAC = angularVelocityPartAC + CVec3D(speedA);
	CVec3D relativeVelocityC = velocityAC;

	const double normalRelativeVelocityLengthC = -relativeVelocityC.DotProduct(collisionNormalB);
	//CLog::Instance().Stream() << "Impact velocity: " << normalRelativeVelocityLengthC << endl;
	if (normalRelativeVelocityLengthC > -epsilon) {
		resolveImpactStatic(vectorAC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA,
			invertedMassA, invertedAngularMassA, momentumTransferFactorAB);
		resolveSurfaceFrictionStatic(vectorAC, collisionNormalB, relativeVelocityC,
			speedA, angularSpeedA,
			invertedMassA, invertedAngularMassA, surfaceFrictionFactorAB);
		collisionDeltaSpeed += normalRelativeVelocityLengthC;
	}
	pushBackBodiesStatic(collisionInfo.Normal, collisionInfo.Depth, positionA, rotatedRect);

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
