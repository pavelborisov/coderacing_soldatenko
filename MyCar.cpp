#include "MyCar.h"

#include "math.h"

CMyCar::CMyCar() :
	Angle(0),
	AngularSpeed(0),
	MedianAngularSpeed(0),
	EnginePower(0),
	WheelTurn(0),
	Type(0)
{
}

CMyCar::CMyCar(const CMyCar& car) :
	Position(car.Position),
	Speed(car.Speed),
	Angle(car.Angle),
	AngularSpeed(car.AngularSpeed),
	MedianAngularSpeed(car.MedianAngularSpeed),
	EnginePower(car.EnginePower),
	WheelTurn(car.WheelTurn),
	Type(car.Type)
{
}

CMyCar::CMyCar(const model::Car& car, double angularSpeedFactor, const model::Car& previousCar) :
	Position(car.getX(), car.getY()),
	Speed(car.getSpeedX(), car.getSpeedY()),
	Angle(car.getAngle()),
	AngularSpeed(car.getAngularSpeed()),
	// ���� - ������� ������� ����� angularSpeedFactor � ���������� ������
	//MedianAngularSpeed(car.getMedianAngularSpeed),
	EnginePower(car.getEnginePower()),
	WheelTurn(car.getWheelTurn()),
	Type(car.getType())
{
	// TODO: ��������� ���� ������� � ���������. � ���������� �� MedianAngularSpeed � CMyCar
	CVec2D previousDirectionUnitVector(cos(previousCar.getAngle()), sin(previousCar.getAngle()));
	CVec2D previousSpeedVector(previousCar.getSpeedX(), previousCar.getSpeedY());
	MedianAngularSpeed = angularSpeedFactor * previousCar.getWheelTurn()
		* previousDirectionUnitVector.DotProduct(previousSpeedVector);
}
