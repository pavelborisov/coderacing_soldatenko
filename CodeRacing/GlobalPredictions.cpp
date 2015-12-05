#include "GlobalPredictions.h"

using namespace std;

vector<CMyOil> CGlobalPredictions::Oils;
vector<CMyBonus> CGlobalPredictions::Bonuses;
vector<vector<CMyWasher>> CGlobalPredictions::WashersPerTick;
vector<vector<CMyTire>> CGlobalPredictions::TiresPerTick;
vector<vector<CMyCar>> CGlobalPredictions::EnemyCarsPerTick;

void CGlobalPredictions::Clear()
{
	Oils.clear();
	Bonuses.clear();
	WashersPerTick.clear();
	TiresPerTick.clear();
	EnemyCarsPerTick.clear();
}
