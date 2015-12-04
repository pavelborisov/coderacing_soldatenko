#include "Simulator.h"

#include <algorithm>
#include "math.h"
#include "assert.h"
#include "DrawPlugin.h"
#include "Log.h"
#include "MyTile.h"
#include "Tools.h"

using namespace std;

static const int subtickCount = 10;
static const double dTime = 1.0 / subtickCount;

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
	carRotationFrictionFactorDt = game.getCarRotationFrictionFactor() * dTime / 5; // WHY /5 ???

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

CMyCar CSimulator::Predict(const CMyCar& startCar, const model::World& /*world*/, const model::Move& move, int currentTick) const
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
	updateCarPosition(car.Position, car.Speed, car.Angle, car.AngularSpeed, car.MedianAngularSpeed,
		lengthwiseUnitVector, crosswiseUnitVector,
		isBrake ? CVec2D(0, 0) : accelerationDt,
		carMovementAirFrictionFactorDt,
		isBrake && !isOiled ? carCrosswiseFrictionFactorDt : carLengthwiseFrictionFactorDt,
		isOiled ? carLengthwiseFrictionFactorDt : carCrosswiseFrictionFactorDt,
		carRotationAirFrictionFactorDt,
		carRotationFrictionFactorDt);

	car.UpdateRotatedRect();
	processWallCollision(startCar, car);

	return car;
}

CMyTire CSimulator::Predict(const CMyTire& tire, const model::World& world, int currentTick) const
{
	tire;
	world;
	currentTick;
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

void CSimulator::updateCarPosition(
	CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed, double& medianAngularSpeed,
	CVec2D& lengthwiseUnitVector, CVec2D& crosswiseUnitVector, const CVec2D& accelerationDt,
	double movementAirFrictionFactorDt, double lengthwiseFrictionFactorDt, double crosswiseFrictionFactorDt,
	double rotationAirFrictionFactorDt, double rotationFrictionFactorDt) const
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
	}

	normalizeAngle(angle);
}

void CSimulator::updateCirclePosition(
	CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed,
	double movementAirFrictionFactorDt, double movementFrictionFactorDt,
	double rotationAirFrictionFactorDt, double rotationFrictionFactorDt) const
{
	// Физика считается в несколько итераций.
	for (int i = 0; i < subtickCount; i++) {
		position += speed * dTime;

		if(movementAirFrictionFactorDt > 0) speed *= movementAirFrictionFactorDt;
		if (movementFrictionFactorDt > 0) {
			CVec2D speedUnitVector = (speed.X != 0 && speed.Y != 0) ? speed * (1 / speed.Length()) : CVec2D();
			CVec2D frictionVector = speedUnitVector * movementFrictionFactorDt;
			speed.X -= limit(speed.X, frictionVector.X);
			speed.Y -= limit(speed.Y, frictionVector.Y);
		}

		angle += angularSpeed * dTime;

		if(rotationAirFrictionFactorDt > 0) angularSpeed *= carRotationAirFrictionFactorDt;
		if(rotationFrictionFactorDt > 0) angularSpeed -= limit(angularSpeed, carRotationFrictionFactorDt);
	}

	normalizeAngle(angle);
}


void CSimulator::processWallCollision(const CMyCar& startCar, CMyCar& car) const
{
	///////////////
	// Тупейшая обработка коллизий со стенами.
	CMyTile carTile(car.Position);
	assert(carTile.IsCorrect());

	const double tileSize = CMyTile::TileSize;
	CVec2D carTileTopLeft(carTile.X * tileSize, carTile.Y * tileSize);
	CVec2D topLeft(0, 0);
	CVec2D topRight(tileSize, 0);
	CVec2D bottomLeft(0, tileSize);
	CVec2D bottomRight(tileSize, tileSize);
	const double radius = 80;
	const double radiusSqr = radius*radius;
	const bool leftWall = !carTile.IsLeftOpen();
	const bool rightWall = !carTile.IsRightOpen();
	const bool topWall = !carTile.IsTopOpen();
	const bool bottomWall = !carTile.IsBottomOpen();

	bool collision = false;
	for (auto corner : car.RotatedRect.Corners) {
		corner -= carTileTopLeft;
		if (CVec2D(corner - topLeft).LengthSquared() <= radiusSqr ||
			CVec2D(corner - topRight).LengthSquared() <= radiusSqr ||
			CVec2D(corner - bottomLeft).LengthSquared() <= radiusSqr ||
			CVec2D(corner - bottomRight).LengthSquared() <= radiusSqr ||
			(leftWall && corner.X <= radius) ||
			(rightWall && corner.X >= (tileSize - radius)) ||
			(topWall && corner.Y <= radius) ||
			(bottomWall && corner.Y >= (tileSize - radius)))
		{
			collision = true;
			break;
		}
	}
	if (collision) {
		car.Position = startCar.Position;
		car.Angle = startCar.Angle;
		car.Speed.X = 0;
		car.Speed.Y = 0;
		car.AngularSpeed = 0;
		car.CollisionDetected = true;
	}
}
