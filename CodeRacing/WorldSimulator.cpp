#include "WorldSimulator.h"

#include <algorithm>
#include "assert.h"
#include "Tools.h"

using namespace std;

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
	CMyWorld world = startWorld;
	CCarInfo carInfos[CMyWorld::MaxCars];
	for (int i = 0; i < CMyWorld::MaxCars; i++) {
		updateCar(moves[i], world.Cars[i], carInfos[i], world);
	}

	for (int subtick = 0; subtick < subtickCount; subtick++) {
		// Сначала всё двигаем.
		for (int i = 0; i < CMyWorld::MaxCars; i++) {
			moveCar(carInfos[i], world.Cars[i]);
		}
		for (int i = 0; i < CMyWorld::MaxWashers; i++) {
			if (world.Washers[i].IsValid()) {
				moveWasher(world.Washers[i]);
			}
		}
		for (int i = 0; i < CMyWorld::MaxTires; i++) {
			if (world.Tires[i].IsValid()) {
				moveTire(world.Tires[i]);
			}
		}

		//// Теперь разбираемся с коллизиями. Какой правильный порядок - хз.
		//for (int i = 0; i < CMyWorld::MaxTires; i++) {
		//	if (world.Tires[i].IsValid()) {
		//		collideTireWithWalls(world.Tires[i]);
		//		collideTireWithWashers(world.Washers[i]);
		//	}
		//}
		//for (int i = 0; i < CMyWorld::MaxCars; i++) {
		//	collideCarWithWalls(world.Cars[i]);
		//	collideCarWithWashers(world.Cars[i], world);
		//	collideCarWithTires(world.Cars[i], world);
		//	collideCarWithBonuses(world.Cars[i], world);
		//	for (int j = 1; j < CMyWorld::MaxCars; j++) {
		//		collideCarWithCar(world.Cars[i], world.Cars[j]);
		//	}
		//}
	}
}

void CWorldSimulator::updateCar(const CMyMove& move, CMyCar& car, CCarInfo& carInfo, CMyWorld& world) const
{
	carInfo.LengthwiseUnitVector = { cos(car.Angle), sin(car.Angle) };

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
	carInfo.AccelerationDt = car.EnginePower >= 0 ?
		carInfo.LengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
		carInfo.LengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;

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
	// 1. Ускорение.
	car.Speed += carInfo.AccelerationDt;
	// 2. Трение об воздух - пропорционально скорости и равномерно по всем направлениям.
	car.Speed *= carMovementAirFrictionFactorDt;
	// 3. Трение колёс - постоянно и различно по направлениям. Если тормозим, то к продольному направлению надо
	//    применить такое же трение, что и к поперечному.
	const double frictionLengthwise = limit(car.Speed.DotProduct(carInfo.LengthwiseUnitVector), carLengthwiseFrictionFactorDt);
	const double frictionCrosswise = limit(car.Speed.DotProduct(carInfo.CrosswiseUnitVector), carCrosswiseFrictionFactorDt);
	car.Speed -= carInfo.LengthwiseUnitVector * frictionLengthwise + carInfo.CrosswiseUnitVector * frictionCrosswise;

	// Обновление угла.
	car.Angle += car.AngularSpeed * dTime;
	carInfo.LengthwiseUnitVector = CVec2D(cos(car.Angle), sin(car.Angle));
	carInfo.CrosswiseUnitVector = CVec2D(carInfo.LengthwiseUnitVector.Y, -carInfo.LengthwiseUnitVector.X);

	// Обновление угловой скорости.
	// Все трения применяются к той части, которая отличается от базовой скорости. Сначала воздушное трение, потом общее трение.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.AngularSpeed *= carRotationAirFrictionFactorDt;
	car.AngularSpeed -= limit(car.AngularSpeed, carRotationFrictionFactorDt);
	car.AngularSpeed += car.MedianAngularSpeed;

	normalizeAngle(car.Angle);
}

void CWorldSimulator::moveWasher(CMyWasher& washer) const
{
	washer.Position += washer.Speed * dTime;
}

void CWorldSimulator::moveTire(CMyTire& tire) const
{
	tire.Position += tire.Speed * dTime;
}
