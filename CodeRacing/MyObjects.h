#pragma once

#include "model\Bonus.h"
#include "model\OilSlick.h"
#include "model\Projectile.h"
#include "Vec2D.h"

struct CMyOil {
	CVec2D Position;
	int LastTick = 0; // TODO: Убрать

	static const double Radius;

	CMyOil();
	CMyOil(const model::OilSlick& oil);
};

struct CMyBonus {
	CVec2D Position;
	model::BonusType Type = model::BonusType::_UNKNOWN_BONUS_TYPE_;

	CMyBonus();
	explicit CMyBonus(const model::Bonus& bonus);
};

struct CMyWasher {
	int CarId = -1;
	CVec2D Position;
	CVec2D Speed;

	static const double Radius;
	static const double Mass;
	
	CMyWasher();
	explicit CMyWasher(const model::Projectile& projectile, int carId);
	bool IsValid() const { return CarId >= 0; }
	void Invalidate() { CarId = -1; }
};

struct CMyTire {
	int CarId = -1;
	CVec2D Position;
	CVec2D Speed;
	double AngularSpeed = 0;

	static const double Radius;
	static const double Mass;
	static const double InvertedMass;
	static const double AngularMass;
	static const double InvertedAngularMass;

	CMyTire();
	explicit CMyTire(const model::Projectile& projectile, int carId);
	bool IsValid() const { return CarId >= 0; }
	void Invalidate() { CarId = -1; }
};
