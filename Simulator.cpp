#include "Simulator.h"

#include <algorithm>
#include "math.h"
#include "assert.h"

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
	rotationAirFrictionFactorDt(0),
	powerChangePerTick(0),
	wheelTurnChangePerTick(0),
	angularSpeedFactor(0)
{
}

void CSimulator::Initialize(const model::Game& game)
{
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

	powerChangePerTick = game.getCarEnginePowerChangePerTick();
	wheelTurnChangePerTick = game.getCarWheelTurnChangePerTick();
	angularSpeedFactor = game.getCarAngularSpeedFactor();

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
	CMyCar car(startCar);

	// ��������� ������ ������������ ����, ���� ������� ����������.
	CVec2D lengthwiseUnitVector(cos(car.Angle), sin(car.Angle));
	// ������������� �����������.
	CVec2D crosswiseUnitVector(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

	// ��������� �������� ���������.
	car.EnginePower += limit(move.getEnginePower() - car.EnginePower, powerChangePerTick);
	car.EnginePower = limit(car.EnginePower, 1.0); // TODO: ����� ����� ����������� ����� - ���� �������� �����������.
	// ������ ���������. ����� ���������� ��� ���� �������� ������.
	CVec2D accelerationDt = car.EnginePower >= 0 ?
		lengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime:
		lengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;

	// ��������� ���� �������� ����.
	car.WheelTurn += limit(move.getWheelTurn() - car.WheelTurn, wheelTurnChangePerTick);
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// ������� ��������� "�������" ������� ��������. ��� ����� ���� �� ��� �������.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.MedianAngularSpeed = angularSpeedFactor * car.WheelTurn * lengthwiseUnitVector.DotProduct(car.Speed);
	car.AngularSpeed += car.MedianAngularSpeed;

	// ������ ��������� � ��������� ��������.
	for (int i = 0; i < subtickCount; i++) {
		// ���������� �������.
		car.Position += car.Speed * dTime;

		// ���������� ��������.
		// 1. ���������.
		car.Speed += accelerationDt;
		// 2. ������ �� ������ - ��������������� �������� � ���������� �� ���� ������������.
		car.Speed *= movementAirFrictionFactorDt;
		// 3. ������ ���� - ��������� � �������� �� ������������. ���� ��������, �� � ����������� ����������� ����
		//    ��������� ����� �� ������, ��� � � �����������.
		const double frictionLengthwise = limit(car.Speed.DotProduct(lengthwiseUnitVector),
			move.isBrake() ? crosswiseFrictionFactorDt : lengthwiseFrictionFactorDt );
		const double frictionCrosswise = limit(car.Speed.DotProduct(crosswiseUnitVector), crosswiseFrictionFactorDt);
		car.Speed -= lengthwiseUnitVector * frictionLengthwise + crosswiseUnitVector * frictionCrosswise;

		// ���������� ����.
		car.Angle += car.AngularSpeed * dTime;
		lengthwiseUnitVector = CVec2D(cos(car.Angle), sin(car.Angle));
		crosswiseUnitVector = CVec2D(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

		// ���������� ������� ��������.
		// TODO: ������ rotationFrictionFactorDt
		car.AngularSpeed = car.MedianAngularSpeed + (car.AngularSpeed - car.MedianAngularSpeed)
			* rotationAirFrictionFactorDt;
	}

	return car;
}
