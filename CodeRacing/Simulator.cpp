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

CMyCar CSimulator::Predict(const CMyCar& startCar, const model::Move& move, int currentTick) const
{
	// TODO: ��������. �� �������, �������, ������� ��������, ��������(!)
	// TODO: ��������� �������, ���� �� ������
	// TODO: ��������� ��� ������� � ��� �������
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
		carRotationAirFrictionFactorDt, carRotationFrictionFactorDt,
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

	// �������� ��������.
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

	// �����.
	if (move.isUseNitro() && !isDead) {
		assert(car.NitroCount > 0 && car.NitroTicks == 0 && car.NitroCooldown == 0);
		car.NitroCount--;
		car.NitroTicks = game.getNitroDurationTicks();
		car.NitroCooldown = game.getUseNitroCooldownTicks();
	}
	const bool isNitro = car.NitroTicks > 0 && !isDead;
	car.NitroTicks = max(0, car.NitroTicks - 1);
	car.NitroCooldown = max(0, car.NitroCooldown - 1);

	// ����
	for (const auto& o : CGlobalPredictions::Oils) {
		if (currentTick >= o.LastTick || car.OiledTicks > 0) continue;
		// TODO: ��������� ������������ ���� ��� ������ � ��.
		if ((car.Position - o.Position).LengthSquared() <= 150 * 150) {
			car.OiledTicks = min(o.LastTick - currentTick, game.getMaxOiledStateDurationTicks());
		}
	}
	isOiled = car.OiledTicks > 0;
	car.OiledTicks = max(0, car.OiledTicks - 1);

	// ������
	isBrake = move.isBrake() && !isDead;

	// ������� �� ��������� � �� ����.
	const double enginePower = isDead ? 0 : move.getEnginePower();
	const double wheelTurn = isDead ? car.WheelTurn : move.getWheelTurn();

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
	accelerationDt = car.EnginePower >= 0 ?
		lengthwiseUnitVector * forwardAccelByType[car.Type] * car.EnginePower * dTime :
		lengthwiseUnitVector * rearAccelByType[car.Type] * car.EnginePower * dTime;

	// ��������� ���� �������� ���� � ������� (���������) ��������.
	if (car.MedianAngularSpeed == UndefinedMedianAngularSpeed) {
		// ������� ������� ������� ��������, ���� ������ ���� ������� �� model::Car - ��� ���� ������ ��� :(
		// � ���������, ����� ��� �������� � ��������� ������ �������� ��� ������������ ������� �� ������ ����.
		car.MedianAngularSpeed = game.getCarAngularSpeedFactor() * car.WheelTurn * lengthwiseUnitVector.DotProduct(car.Speed);
	}
	car.WheelTurn += limit(wheelTurn - car.WheelTurn, game.getCarWheelTurnChangePerTick());
	car.WheelTurn = limit(car.WheelTurn, 1.0);
	// ������ ��������� ������� �������� �� ���� ������� ���.
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
	// ������ ��������� � ��������� ��������.
	for (int i = 0; i < subtickCount; i++) {
		// ���������� �������.
		position += speed * dTime;

		// ���������� ��������.
		// 1. ���������.
		speed += accelerationDt;
		// 2. ������ �� ������ - ��������������� �������� � ���������� �� ���� ������������.
		speed *= movementAirFrictionFactorDt;
		// 3. ������ ���� - ��������� � �������� �� ������������. ���� ��������, �� � ����������� ����������� ����
		//    ��������� ����� �� ������, ��� � � �����������.
		const double frictionLengthwise = limit(speed.DotProduct(lengthwiseUnitVector), lengthwiseFrictionFactorDt);
		const double frictionCrosswise = limit(speed.DotProduct(crosswiseUnitVector), crosswiseFrictionFactorDt);
		speed -= lengthwiseUnitVector * frictionLengthwise + crosswiseUnitVector * frictionCrosswise;

		// ���������� ����.
		angle += angularSpeed * dTime;
		lengthwiseUnitVector = CVec2D(cos(angle), sin(angle));
		crosswiseUnitVector = CVec2D(lengthwiseUnitVector.Y, -lengthwiseUnitVector.X);

		// ���������� ������� ��������.
		// ��� ������ ����������� � ��� �����, ������� ���������� �� ������� ��������. ������� ��������� ������, ����� ����� ������.
		angularSpeed -= medianAngularSpeed;
		angularSpeed *= rotationAirFrictionFactorDt;
		angularSpeed -= limit(angularSpeed, rotationFrictionFactorDt);
		angularSpeed += medianAngularSpeed;

		// ��������� ��������.
		if (radius < 0) {
			rotatedRect = CRotatedRect(position, 210, 140, angle);
		}
		if (!passThroughWalls) {
			processWallCollision(position, speed, angle, angularSpeed, radius, rotatedRect);
		}
	}

	normalizeAngle(angle);
}

void CSimulator::processWallCollision(CVec2D& position, CVec2D& speed, double& angle, double& angularSpeed,
	double radius, CRotatedRect& rotatedRect) const
{
	static const double tileSize = 800;
	static const double wallRadius = 80;
	static const double epsilon = 1e-7;
	if (radius > 0) {
		position;
		speed;
		angle;
		angularSpeed;
		radius;
		rotatedRect;

	} else {
		// TODO ������-�� ���� ����� ������ � ������, ��� � �������. � ���� ������������, ���
		// ��� ��������������, ������� �� ����� ���������� - ��� ���� ����������.
		static const double width = 210;
		static const double height = 140;
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
				// TODO: ���������� ���� ���� ������� �� ����� :(
				CMyTile tile(tileX, tileY);
				const double centerOffsetX = position.X - tileX * tileSize;
				const double centerOffsetY = position.Y - tileY * tileSize;
				if (!tile.IsLeftOpen() && wallRadius + biggerRadius > centerOffsetX) {
					const double tileLeftWallX = tileX * tileSize + wallRadius;
					int cornersInsideWall = 0;
					CVec2D cornersSum;
					double depth = 0;
					for (const auto& c : rotatedRect.Corners) {
						if (c.X < tileLeftWallX) {
							cornersInsideWall++;
							cornersSum += c;
							depth = max(depth, tileLeftWallX - c.X);
						}
					}
					if (cornersInsideWall > 0) {
						cornersSum *= 1.0 / cornersInsideWall;
						CVec3D collisionNormalB(1, 0, 0);
						// TODO: ��������, ���� �������� ����� ���� ����� - ��� ��������� ���������� ����� ������������ � �������?
						CVec2D collisionPoint(tileLeftWallX, cornersSum.Y);

						CVec3D vectorAC(collisionPoint - position);
						CVec3D angularVelocityPartAC = CVec3D(angularSpeed).Cross(vectorAC);
						CVec3D velocityAC = angularVelocityPartAC + CVec3D(speed);
						CVec3D relativeVelocityC = velocityAC;
						const double normalRelativeVelocityLengthC = -relativeVelocityC.DotProduct(collisionNormalB);
						CLog::Instance().Stream() << "Corners inside left wall: " << cornersInsideWall << endl;
						CLog::Instance().Stream() << "Impact velocity: " << normalRelativeVelocityLengthC << endl;
						if (normalRelativeVelocityLengthC > -epsilon) {
							const double mass = 1;
							const double invertedMass = 1 / mass;
							const double angularMass = 1.0 / 12 * mass * (width * width + height * height);
							const double invertedAngularMass = 1 / angularMass;
							//resolveImpact
							{
								// TODO: ����������. 
								static const double momentumTransferFactor = 0.25;
								// TODO: ����� ����� ������� ������?
								// ������ ������� ��������������
								CVec3D denominatorPartA = vectorAC.Cross(collisionNormalB);
								denominatorPartA *= invertedAngularMass;
								denominatorPartA = denominatorPartA.Cross(vectorAC);
								const double denominator = invertedMass + collisionNormalB.DotProduct(denominatorPartA);
								const double impulseChange = -(1 + momentumTransferFactor) * relativeVelocityC.DotProduct(collisionNormalB) / denominator;
								CLog::Instance().Stream() << "Impulse change: " << impulseChange << endl;
								//if (impulseChange < epsilon) {
								//	return;
								//}
								if (impulseChange > epsilon) {
									CVec3D velocityChangeA = collisionNormalB * (impulseChange * invertedMass);
									speed = { speed.X + velocityChangeA.X, speed.Y + velocityChangeA.Y };

									CVec3D angularVelocityChangeA = vectorAC.Cross(collisionNormalB * impulseChange) * invertedAngularMass;
									angularSpeed = angularSpeed + angularVelocityChangeA.Z;
								}
							}

							//resolveSurfaceFriction
							{
								CVec3D tangent = relativeVelocityC - (collisionNormalB * (relativeVelocityC.DotProduct(collisionNormalB)));
								//if (tangent.LengthSquared() < epsilon * epsilon) {
								//	return;
								//}
								if (tangent.LengthSquared() > epsilon * epsilon) {
									tangent *= 1.0 / tangent.Length();
									// TODO: ����������
									static const double surfaceFrictionFactorA = 0;
									static const double surfaceFrictionFactorB = 0;
									static const double sqrt2 = sqrt(2);
									static const double surfaceFrictionFactorABSqrt = sqrt2 * sqrt(surfaceFrictionFactorA * surfaceFrictionFactorB);
									const double surfaceFriction = surfaceFrictionFactorABSqrt * abs(relativeVelocityC.DotProduct(collisionNormalB)) / relativeVelocityC.Length();
									//if (surfaceFriction < epsilon) {
									//	return;
									//}
									if (surfaceFriction > epsilon) {
										CVec3D denominatorPartA = vectorAC.Cross(tangent);
										denominatorPartA *= invertedAngularMass;
										denominatorPartA = denominatorPartA.Cross(tangent);
										const double denominator = invertedMass + tangent.DotProduct(denominatorPartA);
										const double impulseChange = -surfaceFriction * relativeVelocityC.DotProduct(tangent) / denominator;
										//if (abs(impulseChange) < epsilon) {
										//	return;
										//}
										if (abs(impulseChange) > epsilon) {
											CVec3D velocityChangeA = tangent * (impulseChange * invertedMass);
											speed = { speed.X + velocityChangeA.X, speed.Y + velocityChangeA.Y };

											CVec3D angularVelocityChangeA = vectorAC.Cross(tangent * impulseChange) * invertedAngularMass;
											angularSpeed = angularSpeed + angularVelocityChangeA.Z;
										}
									}
								}
							}
							//pushBackBodies
							{
								CVec2D collisionNormalB2D(collisionNormalB.X, collisionNormalB.Y);
								position += collisionNormalB2D * (depth + epsilon);;
							}
						}
					}
				}
				if (!tile.IsRightOpen() && centerOffsetX > tileSize - wallRadius - biggerRadius) {

				}
				if (!tile.IsTopOpen() && wallRadius + biggerRadius > centerOffsetY) {

				}
				if (!tile.IsBottomOpen() && centerOffsetY > tileSize - wallRadius - biggerRadius) {

				}
			}
		}
	}
}

