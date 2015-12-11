#include "MyStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <assert.h>
#include "MyWorld.h"
#include "Tools.h"
#include "WaypointsDistanceMap.h"
#include "WorldSimulator.h"
#include <Windows.h>

using namespace model;
using namespace std;

long long MyStrategy::randomSeed = 0;
CBestMoveFinder::CResult MyStrategy::allyResult[2];
int MyStrategy::allyResultTick[2] = { -1, -1 };
const double MyStrategy::stoppedLengthThreshold = 10;
const int MyStrategy::stoppedTicksThreshold = 40;

MyStrategy::MyStrategy() :
	log(CLog::Instance()),
	draw(CDrawPlugin::Instance()),
	currentTick(0),
	nextWaypointIndex(0),
	rear(0),
	stoppedTicks(0)
{
}

void MyStrategy::move(const Car& _self, const World& _world, const Game& _game, Move& _resultMove)
{
	self = &_self;
	world = &_world;
	game = &_game;
	resultMove = &_resultMove;

	if (randomSeed == 0) {
		randomSeed = game->getRandomSeed();
		CLog::Instance().Stream() << "getRandomSeed() == " << randomSeed << endl;
		srand(static_cast<unsigned int>(randomSeed));
	}

	if (self->isFinishedTrack()) {
		return;
	}

	CDrawPluginSwitcher drawSwitcher(draw); 
	currentTick = world->getTick();

	CWorldSimulator::Instance().SetGame(*game);

	currentWorld = CMyWorld(*world, *self);
	currentCar = currentWorld.Cars[0];
	log.LogTick(currentTick);
	log.LogMyCar(currentWorld.Cars[0], "Current            ");

	updateWaypoints();
	findTileRoute();
	makeMove();
	predict();
	doLog();
	doDraw();

	previousPredictedWorld = predictedWorld;
	for (auto& c : predictedWorld.Cars) {
		if (!c.IsValid()) {
			continue;
		}
		c.SaveHistory();
	}
}

void MyStrategy::updateWaypoints()
{
	vector<vector<TileType>> tilesXY = world->getTilesXY();
	bool flush = false;
	if (CMyTile::TileTypesXY.size() == 0) {
		flush = true;
		CMyTile::TileTypesXY = tilesXY;
		CMyTile::TileSize = game->getTrackTileSize();
	}
	for (size_t x = 0; x < tilesXY.size(); x++) {
		for (size_t y = 0; y < tilesXY[0].size(); y++) {
			const model::TileType& src = tilesXY[x][y];
			model::TileType& dst = CMyTile::TileTypesXY[x][y];
			if (src != UNKNOWN && src != dst) {
				flush = true;
				dst = src;
			}
		}
	}

	vector<vector<int>> waypoints = world->getWaypoints();
	waypointTiles.clear();
	for (const auto& w : waypoints) {
		waypointTiles.push_back(CMyTile(w[0], w[1]));
	}

	if (flush) {
		CMyTile::FillWalls();
		CWaypointDistanceMap::Instance().Initialize(waypointTiles);
	}

	nextWaypointIndex = self->getNextWaypointIndex();
}

void MyStrategy::findTileRoute()
{
	const int currentX = static_cast<int>(self->getX() / game->getTrackTileSize());
	const int currentY = static_cast<int>(self->getY() / game->getTrackTileSize());
	currentTile = CMyTile(currentX, currentY);

	int dx = 0;
	int dy = 0;
	if (abs(currentCar.Angle) < PI / 4) {
		dx = 1;
	} else if (abs(currentCar.Angle) > 3 * PI / 4) {
		dx = -1;
	} else if (currentCar.Angle > 0) {
		dy = 1;
	} else if (currentCar.Angle < 0) {
		dy = -1;
	}
	tileRoute = tileRouteFinder.FindRoute(waypointTiles, nextWaypointIndex, currentTile, dx, dy);
}

template<class MAP>
void logStats(const MAP& m, const char* name, std::basic_ostream< char, std::char_traits<char> >& stream)
{
	stream << name << ": ";
	for (auto p : m) {
		stream << p.first << "," << p.second << " ";
	}
	stream << endl;
}

void MyStrategy::makeMove()
{
	bool rearIsBetter = false;
	CWaypointDistanceMap::Instance().Query(currentCar.Position.X, currentCar.Position.Y, currentCar.Angle, currentCar.NextWaypointIndex, rearIsBetter, true);

	if (currentTick < game->getInitialFreezeDurationTicks()) {
		resultMove->setEnginePower(rearIsBetter ? -1.0 : 1.0);
		return;
	}

	CBestMoveFinder::CResult result;
	if (world->getPlayers().size() == 2) {
		const int allyType = 1 - currentCar.Type;
		CBestMoveFinder bestMoveFinder(currentWorld, waypointTiles, previousResult,
			allyResult[allyType], allyResultTick[allyType] != currentTick);
		result = bestMoveFinder.Process(rearIsBetter);
		previousResult = result;
		allyResult[currentCar.Type] = result;
		allyResultTick[currentCar.Type] = currentTick;
		*resultMove = result.CurrentMove.Convert();
	} else {
		CBestMoveFinder bestMoveFinder(currentWorld, waypointTiles, previousResult);
		result = bestMoveFinder.Process(rearIsBetter);
		previousResult = result;
		*resultMove = result.CurrentMove.Convert();
	}

	// Считаем, сколько тиков мы были "на месте"
	double stoppedLength = (currentCar.Position - stoppedPosition).Length();
	if (stoppedLength > stoppedLengthThreshold) {
		stoppedTicks = 0;
		stoppedPosition = currentCar.Position;
	} else {
		stoppedTicks += 1;
	}

	// Тупой задний ход
	double angleToTarget = (tileRoute[1].ToVec() - currentCar.Position).GetAngle();
	double angle = angleToTarget - currentCar.Angle;
	normalizeAngle(angle);
	// Когда симулятор хз что делать.
	if (!result.Success || result.MoveList.back().End < 10) {
		CDrawPlugin::Instance().FillCircle(currentCar.Position.X, currentCar.Position.Y, 50, 0x888888);
		resultMove->setEnginePower(1.0);
		resultMove->setWheelTurn(angle * 32 / PI);
	}
	if (rear == 0) {
		if (self->getDurability() == 0) {
			rear = -game->getCarReactivationTimeTicks() - 50;
		} else if (world->getTick() > 200 && currentCar.Speed.Length() < 1 && stoppedTicks > stoppedTicksThreshold) {
			rear = 120 + static_cast<int>(self->getEnginePower() / game->getCarEnginePowerChangePerTick());
		}
	} else if (rear < 0) {
		rear++;
	} else if (rear > 0) {
		resultMove->setUseNitro(false);
		resultMove->setBrake(false);
		CDrawPlugin::Instance().FillCircle(currentCar.Position.X, currentCar.Position.Y, 50, 0x880088);
		if (rear < 40) {
			resultMove->setEnginePower(0);
			resultMove->setBrake(true);
			resultMove->setWheelTurn(0);
		} else {
			if (currentCar.EnginePower > 0) resultMove->setBrake(true);
			resultMove->setEnginePower(-1.0);
			resultMove->setWheelTurn(angle > 0 ? -1 : 1);
		}
		rear--;
		if (rear == 0) rear = -120;
		stoppedTicks = 0;
	}
}

void MyStrategy::predict()
{
	CMyMove moves[4];
	moves[0] = CMyMove(*resultMove);
	moves[1].Engine = 0;
	moves[2].Engine = 0;
	moves[3].Engine = 0;
	CWorldSimulator::Instance().SetPrecision(10);
	CWorldSimulator::Instance().SetOptions(false, false, false);
	predictedWorld = CWorldSimulator::Instance().Simulate(currentWorld, moves);
}

void MyStrategy::doLog()
{
#ifdef LOGGING
	log.LogMyCar(predictedWorld.Cars[0], "Prediction         ");

	if (currentTick > 180) {
		previousPredictedWorld.LogDifference(currentWorld);
	}
	saveMap();
#endif
}

void MyStrategy::doDraw()
{
#ifdef LOGGING
	CVec2D nextWaypoint = waypointTiles[nextWaypointIndex].ToVec();
	draw.FillCircle(nextWaypoint.X, nextWaypoint.Y, 50, 0xFF0000);

	//const double angle = currentCar.Angle;
	//const int nwp = currentCar.NextWaypointIndex;
	//for (int xs = 0; xs < 2 * CMyTile::SizeX(); xs++) {
	//	for (int ys = 0; ys < 2 * CMyTile::SizeY(); ys++) {
	//		const double x = xs * 400 + 200;
	//		const double y = ys * 400 + 200;
	//		const double dist = CWaypointDistanceMap::Instance().Query(x, y, angle, nwp);
	//		CDrawPlugin::Instance().Text(x, y, to_string(dist).c_str(), 0x000000);
	//	}
	//}
	bool rearIsBetter = false;
	CWaypointDistanceMap::Instance().Query(currentCar.Position.X, currentCar.Position.Y, currentCar.Angle, currentCar.NextWaypointIndex, rearIsBetter, true);
	//CWaypointDistanceMap::Instance().Query(currentCar.Position.X, currentCar.Position.Y, currentCar.Speed.GetAngle(), currentCar.NextWaypointIndex, true);
#endif
}

#ifdef LOGGING
#include <fstream>
#include <codecvt>
#include <locale>
wstring getTileSymbol(model::TileType tileType)
{
	switch (tileType) {
	case EMPTY:
		return L"█";//L'\x2558';// L'█';
	case VERTICAL:
		return L"║"; //L'\x2551';// L'║';
	case HORIZONTAL:
		return L"═"; //L'\x2550';// L'═';
	case LEFT_TOP_CORNER:
		return L"╔"; //L'\x2554';// L'╔';
	case RIGHT_TOP_CORNER:
		return L"╗"; //L'\x2557';// L'╗';
	case LEFT_BOTTOM_CORNER:
		return L"╚"; //L'\x255a';// L'╚';
	case RIGHT_BOTTOM_CORNER:
		return L"╝"; //L'\x255d';// L'╝';
	case LEFT_HEADED_T:
		return L"╣"; //L'\x2560';// L'╠';
	case RIGHT_HEADED_T:
		return L"╠"; //L'\x2563';// '╣';
	case TOP_HEADED_T:
		return L"╩"; //L'\x2566';// L'╦';
	case BOTTOM_HEADED_T:
		return L"╦"; //L'\x2569';// L'╩';
	case CROSSROADS:
		return L"╬"; //L'\x256c';// L'╬';
	case UNKNOWN:
		return L"?"; //L'?';
	default:
		return L"E"; //L'E';
	}
}
#endif

void MyStrategy::saveMap()
{
#ifdef LOGGING
	
	wofstream out;
	out.open(L"dump.map");
	locale utf8_locale(locale(), new codecvt_utf8<wchar_t>);
	out.imbue(utf8_locale);

	out << CMyTile::SizeX() << L" " << CMyTile::SizeY() << L"\n";
	for (int y = 0; y < CMyTile::SizeY(); y++) {
		for (int x = 0; x < CMyTile::SizeX(); x++) {
			out << getTileSymbol(CMyTile::TileTypesXY[x][y]);
		}
		out << L"\n";
	}
	out << currentWorld.WaypointTiles.size() << L"\n";
	for (const auto& w : currentWorld.WaypointTiles) {
		out << w.X << L" " << w.Y << L"\n";
	}
	switch (currentWorld.StartDirection) {
	case D_Right:
		out << L"RIGHT\n";
		break;
	case D_Left:
		out << L"LEFT\n";
		break;
	case D_Top:
		out << L"UP\n";
		break;
	case D_Bot:
		out << L"DOWN\n";
		break;
	default:
		assert(false);
	}
	out.close();
#endif
}
