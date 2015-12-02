#include "Simulator.h"

#include <algorithm>
#include "math.h"
#include "assert.h"
#include "Log.h"
#include "MyTile.h"
#include "DrawPlugin.h"
#include "Tools.h"

using namespace std;

static const int subtickCount = 2;
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
	rotationFrictionFactorDt = game.getCarRotationFrictionFactor() * dTime / 5; // WHY /5 ???

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
	// TODO: ��������. �� �������, �������, ������� ��������, ��������(!)
	// TODO: ��������� �������, ���� �� ������
	// TODO: ��������� ��� ������� � ��� �������
	CMyCar car(startCar);

	///////////////
	// �������� ��������� �������� �� �������.
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
		//car = startCar;
		car.Speed.X = 0;
		car.Speed.Y = 0;
		car.AngularSpeed = 0;
		car.CollisionDetected = true;
		return car;
	}
	// ���������� ���� ��������.
	car.CollisionDetected = false;
	///////////////

	// ��������� ������ ������������ ����, ���� ������� ����������.
	CVec2D lengthwiseUnitVector(cos(car.Angle), sin(car.Angle));
	// ������������� �����������.
	CVec2D crosswiseUnitVector(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

	// �����.
	if (move.isUseNitro()) {
		assert(car.NitroCount > 0 && car.NitroTicks == 0 && car.NitroCooldown == 0);
		car.NitroCount--;
		car.NitroTicks = game.getNitroDurationTicks();
		car.NitroCooldown = game.getUseNitroCooldownTicks();
	}
	const bool isNitro = car.NitroTicks > 0;
	car.NitroTicks = max(0, car.NitroTicks - 1);
	car.NitroCooldown = max(0, car.NitroCooldown - 1);

	// ����
	const bool isOiled = car.OiledTicks > 0;
	car.OiledTicks = max(0, car.OiledTicks - 1);

	// ������
	const bool isBrake = move.isBrake();

	// ��������� �������� ���������.
	if (isNitro) {
		car.EnginePower = game.getNitroEnginePowerFactor();
	} else {
		// ����� ��������� ����� ���� �� ������ �������� �������� �� 1 ����� ���������� ��������.
		car.EnginePower = limit(car.EnginePower, 1.0);
		car.EnginePower += limit(move.getEnginePower() - car.EnginePower, game.getCarEnginePowerChangePerTick());
		car.EnginePower = limit(car.EnginePower, 1.0);
	}
	// ������ ���������. ����� ���������� ��� ���� �������� ������.
	CVec2D accelerationDt = car.EnginePower >= 0 ?
		lengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
		lengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;

	// ��������� ���� �������� ���� � ������� (���������) ��������.
	if (car.MedianAngularSpeed == UndefinedMedianAngularSpeed) {
		// ������� ������� ������� ��������, ���� ������ ���� ������� �� model::Car - ��� ���� ������ ��� :(
		// � ���������, ����� ��� �������� � ��������� ������ �������� ��� ������������ ������� �� ������ ����.
		car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * lengthwiseUnitVector.DotProduct(car.Speed);
	}
	car.WheelTurn += limit(move.getWheelTurn() - car.WheelTurn, game.getCarWheelTurnChangePerTick());
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// ������ ��������� ������� �������� �� ���� ������� ���.
	car.AngularSpeed -= car.MedianAngularSpeed;
	car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * lengthwiseUnitVector.DotProduct(car.Speed);
	car.AngularSpeed += car.MedianAngularSpeed;

	// ������ ��������� � ��������� ��������.
	for (int i = 0; i < subtickCount; i++) {
		// ���������� �������.
		car.Position += car.Speed * dTime;

		// ���������� ��������.
		// 1. ���������.
		if (!isBrake) {
			car.Speed += accelerationDt;
		}
		// 2. ������ �� ������ - ��������������� �������� � ���������� �� ���� ������������.
		car.Speed *= movementAirFrictionFactorDt;
		// 3. ������ ���� - ��������� � �������� �� ������������. ���� ��������, �� � ����������� ����������� ����
		//    ��������� ����� �� ������, ��� � � �����������.
		const double frictionLengthwise = limit(
			car.Speed.DotProduct(lengthwiseUnitVector),
			isBrake && !isOiled ? crosswiseFrictionFactorDt : lengthwiseFrictionFactorDt);
		const double frictionCrosswise = limit(
			car.Speed.DotProduct(crosswiseUnitVector),
			isOiled ? lengthwiseFrictionFactorDt : crosswiseFrictionFactorDt);
		car.Speed -= lengthwiseUnitVector * frictionLengthwise + crosswiseUnitVector * frictionCrosswise;

		// ���������� ����.
		car.Angle += car.AngularSpeed * dTime;
		lengthwiseUnitVector = CVec2D(cos(car.Angle), sin(car.Angle));
		crosswiseUnitVector = CVec2D(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

		// ���������� ������� ��������.
		// ��� ������ ����������� � ��� �����, ������� ���������� �� ������� ��������. ������� ��������� ������, ����� ����� ������.
		car.AngularSpeed -= car.MedianAngularSpeed;
		car.AngularSpeed *= rotationAirFrictionFactorDt;
		car.AngularSpeed -= limit(car.AngularSpeed, rotationFrictionFactorDt);
		car.AngularSpeed += car.MedianAngularSpeed;
	}

	normalizeAngle(car.Angle);
	// ������� ������������� ������.
	car.UpdateRotatedRect();

	return car;
}
