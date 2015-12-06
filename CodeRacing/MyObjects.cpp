#include "MyObjects.h"

#include "assert.h"
#include "Log.h"

const double CMyOil::Radius = 300 / 2;
CMyOil::CMyOil()
{
}
CMyOil::CMyOil(const model::OilSlick& oil) :
	Position(oil.getX(), oil.getY())
{
}

const double CMyBonus::Size = 40;
CMyBonus::CMyBonus()
{
}
CMyBonus::CMyBonus(const model::Bonus& bonus) :
	Position(bonus.getX(), bonus.getY()),
	Type(bonus.getType())
{
}


const double CMyWasher::Radius = 40 / 2;
const double CMyWasher::Mass = 10;
const double CMyWasher::InvertedMass = 1.0 / Mass;
const double CMyWasher::AngularMass = 1.0 / 2 * Mass * Radius * Radius;
const double CMyWasher::InvertedAngularMass = 1.0 / AngularMass;
CMyWasher::CMyWasher()
{
}
CMyWasher::CMyWasher(const model::Projectile& projectile, int carId) :
	CarId(carId),
	Position(projectile.getX(), projectile.getY()),
	Speed(projectile.getSpeedX(), projectile.getSpeedY())
{
	assert(projectile.getType() == model::WASHER);
}


const double CMyTire::Radius = 140 / 2;
const double CMyTire::Mass = 1000;
const double CMyTire::InvertedMass = 1.0 / Mass;
const double CMyTire::AngularMass = 1.0 / 2 * Mass * Radius * Radius;
const double CMyTire::InvertedAngularMass = 1.0 / AngularMass;
const double CMyTire::TireToWallMomentumTransferFactor = 0.5;
const double CMyTire::TireToWallSurfaceFrictionFactor = 0.25;
CMyTire::CMyTire()
{
}
CMyTire::CMyTire(const model::Projectile& projectile, int carId) :
	CarId(carId),
	Position(projectile.getX(), projectile.getY()),
	Speed(projectile.getSpeedX(), projectile.getSpeedY()),
	AngularSpeed(projectile.getAngularSpeed())
{
	assert(projectile.getType() == model::TIRE);
}
void CMyTire::LogDifference(const CMyTire& tire) const
{
	CLog::Instance().LogIfDifferent(CarId, tire.CarId, "Tire CarId");
	CLog::Instance().LogIfDifferent(Position.X, tire.Position.X, "Tire Position.X");
	CLog::Instance().LogIfDifferent(Position.Y, tire.Position.Y, "Tire Position.Y");
	CLog::Instance().LogIfDifferent(Speed.X, tire.Speed.X, "Tire Speed.X");
	CLog::Instance().LogIfDifferent(Speed.Y, tire.Speed.Y, "Tire Speed.Y");
	CLog::Instance().LogIfDifferent(AngularSpeed, tire.AngularSpeed, "Tire AngularSpeed");
}


CMyPlayer::CMyPlayer()
{
}
CMyPlayer::CMyPlayer(const model::Player& player) :
	Score(player.getScore()),
	DamageScore(0)
{
}

void CMyPlayer::LogDifference(const CMyPlayer& player) const
{
	CLog::Instance().LogIfDifferent(Score, player.Score, "Score");
}
