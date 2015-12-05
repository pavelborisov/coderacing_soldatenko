#pragma once

#include <vector>
#include "model\Bonus.h"
#include "MyCar.h"
#include "MyObjects.h"
#include "Vec2D.h"

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
