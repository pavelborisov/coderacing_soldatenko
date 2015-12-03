#pragma once

#include <vector>
#include "model\Bonus.h"
#include "MyCar.h"
#include "Vec2D.h"

struct COilPrediction {
	CVec2D Position;
	int LastTick;
};

struct CBonusPrediction {
	CVec2D Position;
	model::BonusType Type;
	int LastTick;
};

struct CWasherPrediction {
	CVec2D Position;
	CVec2D Speed;
};

struct CTirePrediction {
	CVec2D Position;
	CVec2D Speed;
	double AngularSpeed;
};

struct CGlobalPredictions {
	static std::vector<COilPrediction> Oils;
	static std::vector<CBonusPrediction> Bonuses;

	// Tick dependent
	static const int PredictionDepth = 100;
	static std::vector<std::vector<CWasherPrediction>> WashersPerTick;
	static std::vector<std::vector<CTirePrediction>> TiresPerTick;
	static std::vector<std::vector<CMyCar>> EnemyCarsPerTick;

	static void Clear();
};
