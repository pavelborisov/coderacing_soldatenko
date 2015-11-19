#include "Simulator.h"

#include <algorithm>
#include "math.h"
#include "assert.h"
#include "MyTile.h"
#include "DrawPlugin.h"
#include "Tools.h"

using namespace std;

static const int subtickCount = 10;
static const double dTime = 1.0 / subtickCount;

CSimulator::CSimulator() :
	isInitialized(false),
	forwardAccelByType(2, 0),
	rearAccelByType(2, 0),
	lengthwiseFrictionFactorDt(0),
	crosswiseFrictionFactorDt(0),
	rotationFrictionFactorDt(0),
	movementAirFrictionFactorDt(0),
	rotationAirFrictionFactorDt(0)
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

	lengthwiseFrictionFactorDt = game.getCarLengthwiseMovementFrictionFactor() * dTime;
	crosswiseFrictionFactorDt = game.getCarCrosswiseMovementFrictionFactor() * dTime;
	rotationFrictionFactorDt = game.getCarRotationFrictionFactor() * dTime;

	movementAirFrictionFactorDt = pow(1 - game.getCarMovementAirFrictionFactor(), dTime);
	rotationAirFrictionFactorDt = pow(1 - game.getCarRotationAirFrictionFactor(), dTime);

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

CMyCar CSimulator::Predict(const CMyCar& startCar, const model::World& /*world*/, const model::Move& move) const
{
	// TODO: Коллизии. Со стенами, другими машинами, бонусами(!)
	CMyCar car(startCar);

	///////////////
	// Тупейшая обработка коллизий со стенами.
	CMyTile carTile(car.Position);
	if (!carTile.IsCorrect()) {
		car.Speed.X = 0;
		car.Speed.Y = 0;
		car.AngularSpeed = 0;
		car.CollisionDetected = true;
		return car;
	}
	const double tileSize = CMyTile::TileSize;
	CVec2D carTileTopLeft(carTile.X * tileSize, carTile.Y * tileSize);
	CVec2D carRelative = car.Position - carTileTopLeft;
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

	const double halfHeight = game.getCarHeight() / 2;
	const double halfWidth = game.getCarWidth() / 2;
	vector<CVec2D> carCorners(4);
	carCorners[0] = CVec2D(halfWidth, halfHeight);
	carCorners[1] = CVec2D(halfWidth, -halfHeight);
	carCorners[2] = CVec2D(-halfWidth, -halfHeight);
	carCorners[3] = CVec2D(-halfWidth, halfHeight);
	bool collision = false;
	for (auto& corner : carCorners) {
		corner.Rotate(car.Angle);
		corner += carRelative;
		//CDrawPlugin::Instance().FillCircle(carTileTopLeft + corner, 5);
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
		//car = startCar;
		car.Speed.X = 0;
		car.Speed.Y = 0;
		car.AngularSpeed = 0;
		car.CollisionDetected = true;
		return car;
	}
	///////////////

	// Единичный вектор направленный туда, куда смотрит автомобиль.
	CVec2D lengthwiseUnitVector(cos(car.Angle), sin(car.Angle));
	// Ортогональное направление.
	CVec2D crosswiseUnitVector(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

	// Нитро.
	if (move.isUseNitro()) {
		assert(car.NitroCount > 0 && car.NitroTicks == 0 && car.NitroCooldown == 0);
		car.NitroCount--;
		car.NitroTicks = game.getNitroDurationTicks();
		car.NitroCooldown = game.getUseNitroCooldownTicks();
	}
	const bool isNitro = car.NitroTicks > 0;
	car.NitroTicks = max(0, car.NitroTicks - 1); // TODO: проверить
	car.NitroCooldown = max(0, car.NitroCooldown - 1);

	// Обновляем мощность двигателя.
	if (isNitro) {
		car.EnginePower = game.getNitroEnginePowerFactor();
	} else {
		// После окончания нитро надо не забыть обрезать мощность до 1 перед изменением мощности.
		car.EnginePower = limit(car.EnginePower, 1.0);
		car.EnginePower += limit(move.getEnginePower() - car.EnginePower, game.getCarEnginePowerChangePerTick());
		car.EnginePower = limit(car.EnginePower, 1.0);
	}
	// Вектор ускорения. Будет постоянный для всех итераций физики.
	CVec2D accelerationDt = car.EnginePower >= 0 ?
		lengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
		lengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;

	// Обновляем угол поворота колёс.
	car.WheelTurn += limit(move.getWheelTurn() - car.WheelTurn, game.getCarWheelTurnChangePerTick());
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// Сначала считается "базовая" угловая скорость. Она будет одна на все подтики.
	// NOTE: На форуме была какая-то идея типа вычитать из угловой скорости MedianAngularSpeed от прошлого тика.
	// Но если просто посчитать новую "базовую" скорость и сделать начальную угловую скорость равную ей - то всё
	// работает точно.
	double medianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn
		* lengthwiseUnitVector.DotProduct(car.Speed);
	car.AngularSpeed = medianAngularSpeed;

	// Физика считается в несколько итераций.
	for (int i = 0; i < subtickCount; i++) {
		// Обновление позиции.
		car.Position += car.Speed * dTime;

		// Обновление скорости.
		// 1. Ускорение.
		if (!move.isBrake()) {
			car.Speed += accelerationDt;
		}
		// 2. Трение об воздух - пропорционально скорости и равномерно по всем направлениям.
		car.Speed *= movementAirFrictionFactorDt;
		// 3. Трение колёс - постоянно и различно по направлениям. Если тормозим, то к продольному направлению надо
		//    применить такое же трение, что и к поперечному.
		const double frictionLengthwise = limit(car.Speed.DotProduct(lengthwiseUnitVector),
			move.isBrake() ? crosswiseFrictionFactorDt : lengthwiseFrictionFactorDt);
		const double frictionCrosswise = limit(car.Speed.DotProduct(crosswiseUnitVector), crosswiseFrictionFactorDt);
		car.Speed -= lengthwiseUnitVector * frictionLengthwise + crosswiseUnitVector * frictionCrosswise;

		// Обновление угла.
		car.Angle += car.AngularSpeed * dTime;
		lengthwiseUnitVector = CVec2D(cos(car.Angle), sin(car.Angle));
		crosswiseUnitVector = CVec2D(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

		// Обновление угловой скорости.
		// TODO: Как учесть rotationFrictionFactorDt?
		car.AngularSpeed = medianAngularSpeed + (car.AngularSpeed - medianAngularSpeed) * rotationAirFrictionFactorDt;
	}

	// Вот такая странная обрезка угла происходит движком. Симулируем поведение движка.
	if (car.Angle < -PI) {
		car.Angle += 2 * PI;
	} else if (car.Angle > PI) {
		car.Angle -= 2 * PI;
	}

	return car;
}
