#pragma once

#include <vector>
#include "model\Bonus.h"
#include "MyCar.h"
#include "Vec2D.h"

struct CMyOil {
	CVec2D Position;
	int LastTick;
};

struct CMyBonus {
	CVec2D Position;
	model::BonusType Type;
	int LastTick;
};

struct CMyWasher {
	CVec2D Position;
	CVec2D Speed;

	static const double Radius;
};

struct CMyTire {
	CVec2D Position;
	CVec2D Speed;
	double AngularSpeed;

	static const double Radius;
};

struct CGlobalPredictions {
	static std::vector<CMyOil> Oils;
	static std::vector<CMyBonus> Bonuses;

	// Tick dependent
	static const int PredictionDepth = 100;
	static std::vector<std::vector<CMyWasher>> WashersPerTick;
	static std::vector<std::vector<CMyTire>> TiresPerTick;
	static std::vector<std::vector<CMyCar>> EnemyCarsPerTick;

	static void Clear();
};
