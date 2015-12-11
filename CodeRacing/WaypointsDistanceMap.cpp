#include "WaypointsDistanceMap.h"

#ifdef LOGGING
#undef NDEBUG
#endif

#include <cstdlib>
#include <string>
#include <assert.h>
#include "DrawPlugin.h"
#include "Log.h"
#include "MyWorld.h"

using namespace std;

const unsigned int Left = 1 << 0;
const unsigned int Right = 1 << 1;
const unsigned int Top = 1 << 2;
const unsigned int Bottom = 1 << 3;

const int MaskDX[] = { 1, 1, 0, -1, -1, -1, 0, 1 };
const int MaskDY[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

const double CWaypointDistanceMap::undefinedDistance = INT_MAX;

bool operator == (const CWaypointDistanceMap::CLowResTile& a, const CWaypointDistanceMap::CLowResTile& b)
{
	return a.X == b.X && a.Y == b.Y && a.Direction == b.Direction;
}

bool operator != (const CWaypointDistanceMap::CLowResTile& a, const CWaypointDistanceMap::CLowResTile& b)
{
	return a.X != b.X || a.Y != b.Y || a.Direction != b.Direction;
}

bool CWaypointDistanceMap::CLowResTileWithScore::operator > (const CWaypointDistanceMap::CLowResTileWithScore& b) const
{
	if (Score != b.Score) {
		return Score > b.Score;
	}
	if (LRTile.X != b.LRTile.X) {
		return LRTile.X > b.LRTile.X;
	}
	if (LRTile.Y != b.LRTile.Y) {
		return LRTile.Y > b.LRTile.Y;
	}
	if (LRTile.Direction != b.LRTile.Direction) {
		return LRTile.Direction > b.LRTile.Direction;
	}
	return false;
}

static bool checkOpen(
	const CWaypointDistanceMap::CLowResTile& lrTile,
	int ndx, int ndy,
	const vector<vector<CWaypointDistanceMap::CLowResTile>>& lrTiles)
{
	const int nx = lrTile.X + ndx;
	const int ny = lrTile.Y + ndy;
	if (nx < 0 || nx >= static_cast<int>(lrTiles.size()) || ny < 0 || ny >= static_cast<int>(lrTiles[0].size())) {
		return false;
	}

	assert(abs(ndx) <= 1 && abs(ndy) <= 1);

	const CWaypointDistanceMap::CLowResTile& nlrTile = lrTiles[nx][ny];

	if (ndx == 1 && ndy == 0) {
		return (lrTile.GatesMask & Right) != 0 && (nlrTile.GatesMask & Left) != 0;
	} else if (ndx == -1 && ndy == 0) {
		return (lrTile.GatesMask & Left) != 0 && (nlrTile.GatesMask & Right) != 0;
	} else if (ndx == 0 && ndy == 1) {
		return (lrTile.GatesMask & Bottom) != 0 && (nlrTile.GatesMask & Top) != 0;
	} else if (ndx == 0 && ndy == -1) {
		return (lrTile.GatesMask & Top) != 0 && (nlrTile.GatesMask & Bottom) != 0;
	} else if (ndx == 1 && ndy == 1) {
		return ((lrTile.GatesMask & Right) != 0 && (nlrTile.GatesMask & Top) != 0)
			&& ((lrTile.GatesMask & Bottom) != 0 && (nlrTile.GatesMask & Left) != 0);
	} else if (ndx == -1 && ndy == 1) {
		return ((lrTile.GatesMask & Left) != 0 && (nlrTile.GatesMask & Top) != 0)
			&& ((lrTile.GatesMask & Bottom) != 0 && (nlrTile.GatesMask & Right) != 0);
	} else if (ndx == 1 && ndy == -1) {
		return ((lrTile.GatesMask & Right) != 0 && (nlrTile.GatesMask & Bottom) != 0)
			&& ((lrTile.GatesMask & Top) != 0 && (nlrTile.GatesMask & Left) != 0);
	} else if (ndx == -1 && ndy == -1) {
		return ((lrTile.GatesMask & Left) != 0 && (nlrTile.GatesMask & Bottom) != 0)
			&& ((lrTile.GatesMask & Top) != 0 && (nlrTile.GatesMask & Right) != 0);
	}
	assert(false);
	return false;
}

static void pushN(const CWaypointDistanceMap::CLowResTile& lrTile, int nx, int ny, TDirection d,
	const vector<vector<CWaypointDistanceMap::CLowResTile>>& lrTiles, vector<pair<CWaypointDistanceMap::CLowResTile, double>>& n,
	double dist)
{
	if (checkOpen(lrTile, nx - lrTile.X, ny - lrTile.Y, lrTiles)) {
		n.emplace_back(make_pair(CWaypointDistanceMap::CLowResTile(nx, ny, d, lrTiles[nx][ny].GatesMask), dist));
	}
}

static vector<pair<CWaypointDistanceMap::CLowResTile, double>> findNeighbors(
	const CWaypointDistanceMap::CLowResTile& lrTile,
	const vector<vector<CWaypointDistanceMap::CLowResTile>>& lrTiles)
{
	int x = lrTile.X;
	int y = lrTile.Y;
	vector<pair<CWaypointDistanceMap::CLowResTile, double>> neighbors;

	static const double forwardStraight = 1;
	static const double forwardDiag = 1 * sqrt(2);
	static const double rearStraight = 2;
	static const double rearDiag = 2 * sqrt(2);
	switch(lrTile.Direction) {
	case D_Right:
		// forward
		pushN(lrTile, x - 1, y, D_Right, lrTiles, neighbors, forwardStraight);
		// turns
		pushN(lrTile, x - 1, y - 1, D_RightBot, lrTiles, neighbors, forwardDiag);
		pushN(lrTile, x - 1, y + 1, D_RightTop, lrTiles, neighbors, forwardDiag);
		// rear
		pushN(lrTile, x + 1, y, D_Right, lrTiles, neighbors, rearStraight);
		// rear turns
		pushN(lrTile, x + 1, y + 1, D_RightBot, lrTiles, neighbors, rearDiag);
		pushN(lrTile, x + 1, y - 1, D_RightTop, lrTiles, neighbors, rearDiag);
		break;
	case D_Left:
		// forward
		pushN(lrTile, x + 1, y, D_Left, lrTiles, neighbors, forwardStraight);
		// turns
		pushN(lrTile, x + 1, y - 1, D_LeftBot, lrTiles, neighbors, forwardDiag);
		pushN(lrTile, x + 1, y + 1, D_LeftTop, lrTiles, neighbors, forwardDiag);
		// rear
		pushN(lrTile, x - 1, y, D_Left, lrTiles, neighbors, rearStraight);
		// rear turns
		pushN(lrTile, x - 1, y + 1, D_LeftBot, lrTiles, neighbors, rearDiag);
		pushN(lrTile, x - 1, y - 1, D_LeftTop, lrTiles, neighbors, rearDiag);
		break;
	case D_Bot:
		// forward
		pushN(lrTile, x, y - 1, D_Bot, lrTiles, neighbors, forwardStraight);
		// turns
		pushN(lrTile, x - 1, y - 1, D_RightBot, lrTiles, neighbors, forwardDiag);
		pushN(lrTile, x + 1, y - 1, D_LeftBot, lrTiles, neighbors, forwardDiag);
		// rear
		pushN(lrTile, x, y + 1, D_Bot, lrTiles, neighbors, rearStraight);
		// rear turns
		pushN(lrTile, x + 1, y + 1, D_RightBot, lrTiles, neighbors, rearDiag);
		pushN(lrTile, x - 1, y + 1, D_LeftBot, lrTiles, neighbors, rearDiag);
		break;
	case D_Top:
		// forward
		pushN(lrTile, x, y + 1, D_Top, lrTiles, neighbors, forwardStraight);
		// turns
		pushN(lrTile, x - 1, y + 1, D_RightTop, lrTiles, neighbors, forwardDiag);
		pushN(lrTile, x + 1, y + 1, D_LeftTop, lrTiles, neighbors, forwardDiag);
		// rear
		pushN(lrTile, x, y - 1, D_Top, lrTiles, neighbors, rearStraight);
		// rear turns
		pushN(lrTile, x + 1, y - 1, D_RightTop, lrTiles, neighbors, rearDiag);
		pushN(lrTile, x - 1, y - 1, D_LeftTop, lrTiles, neighbors, rearDiag);
		break;
	case D_RightBot:
		// forward
		pushN(lrTile, x - 1, y - 1, D_RightBot, lrTiles, neighbors, forwardDiag);
		// turns
		pushN(lrTile, x - 1, y, D_Right, lrTiles, neighbors, forwardStraight);
		pushN(lrTile, x, y - 1, D_Bot, lrTiles, neighbors, forwardStraight);
		// rear
		pushN(lrTile, x + 1, y + 1, D_RightBot, lrTiles, neighbors, rearDiag);
		// rear turns
		pushN(lrTile, x + 1, y, D_Right, lrTiles, neighbors, rearStraight);
		pushN(lrTile, x, y + 1, D_Bot, lrTiles, neighbors, rearStraight);
		break;
	case D_RightTop:
		// forward
		pushN(lrTile, x - 1, y + 1, D_RightTop, lrTiles, neighbors, forwardDiag);
		// turns
		pushN(lrTile, x - 1, y, D_Right, lrTiles, neighbors, forwardStraight);
		pushN(lrTile, x, y + 1, D_Top, lrTiles, neighbors, forwardStraight);
		// rear
		pushN(lrTile, x + 1, y - 1, D_RightTop, lrTiles, neighbors, rearDiag);
		// rear turns
		pushN(lrTile, x + 1, y, D_Right, lrTiles, neighbors, rearStraight);
		pushN(lrTile, x, y - 1, D_Top, lrTiles, neighbors, rearStraight);
		break;
	case D_LeftBot:
		// forward
		pushN(lrTile, x + 1, y - 1, D_LeftBot, lrTiles, neighbors, forwardDiag);
		// turns
		pushN(lrTile, x + 1, y, D_Left, lrTiles, neighbors, forwardStraight);
		pushN(lrTile, x, y - 1, D_Bot, lrTiles, neighbors, forwardStraight);
		// rear
		pushN(lrTile, x - 1, y + 1, D_LeftBot, lrTiles, neighbors, rearDiag);
		// rear turns
		pushN(lrTile, x - 1, y, D_Left, lrTiles, neighbors, rearStraight);
		pushN(lrTile, x, y + 1, D_Bot, lrTiles, neighbors, rearStraight);
		break;
	case D_LeftTop:
		// forward
		pushN(lrTile, x + 1, y + 1, D_LeftTop, lrTiles, neighbors, forwardDiag);
		// turns
		pushN(lrTile, x + 1, y, D_Left, lrTiles, neighbors, forwardStraight);
		pushN(lrTile, x, y + 1, D_Top, lrTiles, neighbors, forwardStraight);
		// rear
		pushN(lrTile, x - 1, y - 1, D_LeftTop, lrTiles, neighbors, rearDiag);
		// rear turns
		pushN(lrTile, x - 1, y, D_Left, lrTiles, neighbors, rearStraight);
		pushN(lrTile, x, y - 1, D_Top, lrTiles, neighbors, rearStraight);
		break;
	default:
		assert(false);
	}
	return neighbors;
}

CWaypointDistanceMap::CData::CData(int sizeX, int sizeY, int waypointIndex, CWaypointDistanceMap& wpDistanceMap)
{
	ClosedSet.assign(sizeX, vector<vector<bool>>(sizeY, vector<bool>(D_EnumSize, false)));
	OpenSet.assign(sizeX, vector<vector<bool>>(sizeY, vector<bool>(D_EnumSize, false)));
	CameFrom.assign(sizeX, vector<vector<CLowResTile>>(sizeY, vector<CLowResTile>(D_EnumSize)));
	Distance.assign(sizeX, vector<vector<double>>(sizeY, vector<double>(D_EnumSize, undefinedDistance)));

	const vector<vector<CLowResTile>>& lrTiles = wpDistanceMap.lowResTiles;
	const CMyTile& wpTile = wpDistanceMap.waypoints[waypointIndex];
	const int startX = wpTile.X * tileSize / step;
	const int endX = (wpTile.X + 1) * tileSize / step;
	const int startY = wpTile.Y * tileSize / step;
	const int endY = (wpTile.Y + 1) * tileSize / step;
	// Расстояния внутри клетки с 0-м вейпоинтом будем считать равными 0.
	// А с остальными - будем считать расстояниями до следующих вейпоинтов.
	for (int x = startX; x < endX; x++) {
		for (int y = startY; y < endY; y++) {
			for (int d = 0; d < D_EnumSize; d++) {
				if (waypointIndex == 0) {
					Distance[x][y][d] = 0;
				} else {
					const int nextWaypointIndex = (waypointIndex + 1) % wpDistanceMap.waypoints.size();
					CData& nextData = wpDistanceMap.dataByWp[nextWaypointIndex];
					const double distance = wpDistanceMap.findDistance(x, y, static_cast<TDirection>(d), nextData);
					if (distance == undefinedDistance) {
						ClosedSet[x][y][d] = true;
						continue;
					}
					Distance[x][y][d] = distance;
				}
				OpenSet[x][y][d] = true;
				Queue.push(CLowResTileWithScore(CLowResTile(x, y, static_cast<TDirection>(d), lrTiles[x][y].GatesMask), 0));
			}
		}
	}

	// Все клетки без соседей (соотв. EMPTY) добавим в закрытое множество.
	for (size_t x = 0; x < lrTiles.size(); x++) {
		for (size_t y = 0; y < lrTiles[x].size(); y++) {
			for (size_t d = 0; d < D_EnumSize; d++) {
				if (lrTiles[x][y].GatesMask == 0) {
					ClosedSet[x][y][d] = true;
				}
			}
		}
	}
}

void CWaypointDistanceMap::Initialize(const std::vector<CMyTile>& _waypoints)
{
	waypoints = _waypoints;
	const int wpCount = waypoints.size();
	const int tilesSizeX = CMyTile::TileTypesXY.size();
	const int tilesSizeY = CMyTile::TileTypesXY[0].size();
	const int sizeX = tilesSizeX * tileSize / step;
	const int sizeY = tilesSizeY * tileSize / step;

	lowResTiles.assign(sizeX, vector<CLowResTile>(sizeY));
	for (int tileX = 0; tileX < tilesSizeX; tileX++) {
		const int startX = tileX * tileSize / step;
		const int endX = (tileX + 1) * tileSize / step;
		for (int tileY = 0; tileY < tilesSizeY; tileY++) {
			const int startY = tileY * tileSize / step;
			const int endY = (tileY + 1) * tileSize / step;

			const CMyTile tile(tileX, tileY);
			// Предполагаем, что карта корректная и нет "тупиков"
			const bool leftOpen = tileX > 0 && tile.IsLeftOpen();
			const bool rightOpen = tileX < tilesSizeX - 1 && tile.IsRightOpen();
			const bool topOpen = tileY > 0 && tile.IsTopOpen();
			const bool bottomOpen = tileY < tilesSizeY - 1 && tile.IsBottomOpen();

			for (int x = startX; x < endX; x++) {
				for (int y = startY; y < endY; y++) {
					CLowResTile& lrTile = lowResTiles[x][y];
					lrTile.X = x;
					lrTile.Y = y;
					if (tile.Type() == model::TileType::EMPTY) continue;
					if (x > startX || leftOpen) {
						lrTile.GatesMask |= Left;
					}
					if (x < endX - 1 || rightOpen) {
						lrTile.GatesMask |= Right;
					}
					if (y > startY || topOpen) {
						lrTile.GatesMask |= Top;
					}
					if (y < endY - 1 || bottomOpen) {
						lrTile.GatesMask |= Bottom;
					}
				}
			}
		}
	}

	dataByWp.clear();
	dataByWp.assign(wpCount, CData());
	// Заполняем с конца (когда следующий вп = 0) и продолжаем до самого начала (когда следующий вп = 1)
	for (int i = wpCount; i > 0; i--) {
		int wpi = i % wpCount;
		dataByWp[wpi] = CData(sizeX, sizeY, wpi, *this);
	}
	// TODO: сохранять "длину всей трассы" с учётом стартового направления
}

static TDirection angleToDirection(double angle)
{
	static const double PI8 = PI / 8;
	if (-PI8 <= angle && angle < PI8) {
		return D_Right;
	} else if (PI8 <= angle && angle < 3 * PI8) {
		return D_RightBot;
	} else if (3 * PI8 <= angle && angle < 5 * PI8) {
		return D_Bot;
	} else if (5 * PI8 <= angle && angle < 7 * PI8) {
		return D_LeftBot;
	} else if (7 * PI8 <= angle || angle < -7 * PI8) {
		return D_Left;
	} else if (-7 * PI8 <= angle && angle < -5 * PI8) {
		return D_LeftTop;
	} else if (-5 * PI8 <= angle && angle < -3 * PI8) {
		return D_Top;
	} else if (-3 * PI8 <= angle && angle < -PI8) {
		return D_RightTop;
	}
	assert(false);
	return D_Undefined;
}

double CWaypointDistanceMap::Query(double x, double y, double angle, int waypointIndex, bool& rearIsBetter, bool draw)
{
	const double xLowRes = x / step;
	const double yLowRes = y / step;
	const int xLowResInt = static_cast<int>(xLowRes);
	const int yLowResInt = static_cast<int>(yLowRes);
	normalizeAngle(angle);
	const TDirection dir = angleToDirection(angle);
	const double dist = findDistance(xLowResInt, yLowResInt, dir, dataByWp[waypointIndex]);
	const CLowResTile& next = dataByWp[waypointIndex].CameFrom[xLowResInt][yLowResInt][dir];
	{
		const int dx = MaskDX[dir];
		const int dy = MaskDY[dir];
		rearIsBetter = (xLowResInt - dx) == next.X && (yLowResInt - dy) == next.Y;
	}

	double offset = 0;
	static const double sqrt2 = sqrt(2);
	if (dir == D_Right) {
		offset = (xLowResInt + 1) - xLowRes;
		if (rearIsBetter) offset = 1 - offset;
	} else if (dir == D_Bot) {
		offset = (yLowResInt + 1) - yLowRes;
		if (rearIsBetter) offset = 1 - offset;
	} else if (dir == D_Left) {
		offset = xLowRes - xLowResInt;
		if (rearIsBetter) offset = 1 - offset;
	} else if (dir == D_Top) {
		offset = yLowRes - yLowResInt;
		if (rearIsBetter) offset = 1 - offset;
	} else if (dir == D_RightBot) {
		offset = sqrt2 * (((xLowResInt + 1) - xLowRes) + ((yLowResInt + 1) - yLowRes));
		if (rearIsBetter) offset = sqrt2 - offset;
	} else if (dir == D_LeftBot) {
		offset = sqrt2 * (((yLowResInt + 1) - yLowRes) + (xLowRes - xLowResInt));
		if (rearIsBetter) offset = sqrt2 - offset;
	} else if (dir == D_LeftTop) {
		offset = sqrt2 * ((xLowRes - xLowResInt) + (yLowRes - yLowResInt));
		if (rearIsBetter) offset = sqrt2 - offset;
	} else if (dir == D_RightTop) {
		offset = sqrt2 * ((yLowRes - yLowResInt) + ((xLowResInt + 1) - xLowRes));
		if (rearIsBetter) offset = sqrt2 - offset;
	}

	if (draw) {
		CLowResTile lr;
		lr.Direction = dir;
		lr.X = xLowResInt;
		lr.Y = yLowResInt;
		int wpi = waypointIndex;
		for (int i = 0; i < 10; i++) {
			const CData& data = dataByWp[wpi];
			double d = data.Distance[lr.X][lr.Y][lr.Direction];
			int dx = MaskDX[lr.Direction];
			int dy = MaskDY[lr.Direction];
			bool rear = (data.CameFrom[lr.X][lr.Y][lr.Direction].X + dx) == lr.X && (data.CameFrom[lr.X][lr.Y][lr.Direction].Y + dy) == lr.Y;
			CDrawPlugin::Instance().Rect(lr.X * 400, lr.Y * 400, (lr.X + 1) * 400, (lr.Y + 1) * 400, rear ? 0x808000 : 0xFF0000);
			CDrawPlugin::Instance().Text(lr.X * 400 + 200, lr.Y * 400 + 200, to_string(d).c_str(), rear ? 0x808000 : 0xFF0000);
			CDrawPlugin::Instance().Line(lr.X * 400 + 200, lr.Y * 400 + 200, lr.X * 400 + dx * 200 + 200, lr.Y * 400 + dy * 200 + 200, rear ? 0x808000 : 0xFF0000);
			
			lr = data.CameFrom[lr.X][lr.Y][lr.Direction];
			if (lr == CLowResTile()) break;
			int nwpi = (wpi + 1) % waypoints.size();
			if (lr.X / 2 == waypoints[wpi].X && lr.Y / 2 == waypoints[wpi].Y) {
				wpi = nwpi;
			}
		}
	}

	return dist == undefinedDistance ? undefinedDistance : step * (dist + offset);
}

void CWaypointDistanceMap::GetLRTiles(const CMyCar& car, CLowResTile& current, CLowResTile& next, bool& rearIsBetter)
{
	const double xLowRes = car.Position.X / step;
	const double yLowRes = car.Position.Y / step;
	const int xLowResInt = static_cast<int>(xLowRes);
	const int yLowResInt = static_cast<int>(yLowRes);
	const TDirection dir = angleToDirection(car.Angle);
	current = CLowResTile(xLowResInt, yLowResInt, dir, 0U);
	CLowResTile n = dataByWp[car.NextWaypointIndex].CameFrom[xLowResInt][yLowResInt][dir];
	next = n;
	{
		const int dx = MaskDX[dir];
		const int dy = MaskDY[dir];
		rearIsBetter = (xLowResInt - dx) == next.X && (yLowResInt - dy) == next.Y;
	}
}

double CWaypointDistanceMap::LapScore()
{
	double angle = 0;
	switch (CMyWorld::StartDirection) {
	case D_Right:
		angle = 0;
		break;
	case D_Bot:
		angle = PI / 2;
		break;
	case D_Left:
		angle = PI;
		break;
	case D_Top:
		angle = -PI / 2;
		break;
	default:
		assert(false);
	}
	bool rearIsBetterNotUsed;
	return Query(waypoints[0].X * 800 + 400, waypoints[0].Y * 800 + 400, angle, 1, rearIsBetterNotUsed) + 400;
}

double CWaypointDistanceMap::findDistance(int xt, int yt, TDirection dirt, CData& data)
{
	if (data.ClosedSet[xt][yt][dirt]) {
		return data.Distance[xt][yt][dirt];
	}
	while (!data.Queue.empty()) {
		CLowResTileWithScore lrTileWithScore = data.Queue.top();
		const CLowResTile& lrTile = lrTileWithScore.LRTile;
		const int x = lrTile.X;
		const int y = lrTile.Y;
		const TDirection dir = lrTile.Direction;
		data.Queue.pop();
		if (data.ClosedSet[x][y][dir]) {
			// Мы эту клетку обработали уже ранее.
			assert(!data.OpenSet[x][y][dir]);
			continue;
		}
		data.OpenSet[x][y][dir] = false;
		data.ClosedSet[x][y][dir] = true;

		auto neighbors = findNeighbors(lrTile, lowResTiles);
		for (const auto& n : neighbors) {
			processNeighbor(lrTile, n, data);
		}

		if (x == xt && y == yt && dir == dirt) {
			return data.Distance[x][y][dir];
		}
	}
	// Если мы оказались тут, то путь не был найден. Чего не должно быть.
	return undefinedDistance;
}

void CWaypointDistanceMap::processNeighbor(const CLowResTile& from, const pair<CLowResTile, double>& toD, CData& data)
{
	const CLowResTile& to = toD.first;
	if (data.ClosedSet[to.X][to.Y][to.Direction]) {
		return;
	}

	const double dist = data.Distance[from.X][from.Y][from.Direction] + toD.second;
	if (dist < 0) {
		assert(false);
	}

	if (!data.OpenSet[to.X][to.Y][to.Direction]) {
		// Соседа нет в openSet - добавляем.
		data.Queue.push(CLowResTileWithScore(to, dist));
		data.OpenSet[to.X][to.Y][to.Direction] = true;
	} else if (dist >= data.Distance[to.X][to.Y][to.Direction]) {
		assert(data.Distance[to.X][to.Y][to.Direction] != undefinedDistance);
		// Функция не улучшилась - пропускаем.
		return;
	} else {
		// Есть и в openSet и функция улучшилась. Перебирать очередь в поисках элемента и заменять его - дорого.
		// Поэтому просто добавим новый.
		data.Queue.push(CLowResTileWithScore(to, dist));
	}
	data.CameFrom[to.X][to.Y][to.Direction] = from;
	data.Distance[to.X][to.Y][to.Direction] = dist;
}
