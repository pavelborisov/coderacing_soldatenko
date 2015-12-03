#include "GlobalPredictions.h"

using namespace std;

vector<COilPrediction> CGlobalPredictions::Oils;
vector<CBonusPrediction> CGlobalPredictions::Bonuses;
vector<vector<CWasherPrediction>> CGlobalPredictions::WashersPerTick;
vector<vector<CTirePrediction>> CGlobalPredictions::TiresPerTick;
vector<vector<CMyCar>> CGlobalPredictions::EnemyCarsPerTick;

void CGlobalPredictions::Clear()
{
	Oils.clear();
	Bonuses.clear();
	WashersPerTick.clear();
	TiresPerTick.clear();
	EnemyCarsPerTick.clear();
}
