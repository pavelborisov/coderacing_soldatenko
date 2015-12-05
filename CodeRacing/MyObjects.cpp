#include "MyObjects.h"

#include "assert.h"


const double CMyOil::Radius = 300 / 2;
CMyOil::CMyOil()
{
}
CMyOil::CMyOil(const model::OilSlick& oil) :
	Position(oil.getX(), oil.getY()),
	LastTick(oil.getRemainingLifetime())
{
}


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
