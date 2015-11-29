#include "WaypointsDistanceMap.h"

#include <cstdlib>
#include <assert.h>

using namespace std;

const unsigned int Left = 1 << 0;
const unsigned int Right = 1 << 1;
const unsigned int Top = 1 << 2;
const unsigned int Bottom = 1 << 3;

const int MaskDX[] = { 1, 1, 0, -1, -1, -1, 0, 1 };
const int MaskDY[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

bool operator == (const CWaypointDistanceMap::CLowResTile& a, const CWaypointDistanceMap::CLowResTile& b)
{
	return a.X == b.X && a.Y == b.Y && a.Direction == b.Direction;
}

bool operator != (const CWaypointDistanceMap::CLowResTile& a, const CWaypointDistanceMap::CLowResTile& b)
{
	return a.X != b.X || a.Y != b.Y || a.Direction != b.Direction;
}

bool operator > (const CWaypointDistanceMap::CLowResTileWithScore& a, const CWaypointDistanceMap::CLowResTileWithScore& b)
{
	// TODO: ��� ������������ - �������� ��������� �� X � Y � ������ ���������.
	return a.Score > b.Score;
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
	if (ndx == 1) {
		if ((lrTile.GatesMask & Right) == 0 || (nlrTile.GatesMask & Left) == 0) return false;
	} else if (ndx == -1) {
		if ((lrTile.GatesMask & Left) == 0 || (nlrTile.GatesMask & Right) == 0) return false;
	}
	if (ndy == 1) {
		if ((lrTile.GatesMask & Bottom) == 0 || (nlrTile.GatesMask & Top) == 0) return false;
	} else if(ndy == -1) {
		if ((lrTile.GatesMask & Top) == 0 || (nlrTile.GatesMask & Bottom) == 0) return false;
	}
	return true;
}

static TDirection toDirection(int dx, int dy)
{
	if (dx == 1) {
		if (dy == 0) return D_Right;
		if (dy == 1) return D_RightBot;
		if (dy == -1) return D_RightTop;
	} else if (dx == -1) {
		if (dy == 0) return D_Left;
		if (dy == 1) return D_LeftBot;
		if (dy == -1) return D_LeftTop;
	} else {
		if (dy == 1) return D_Bot;
		if (dy == -1) return D_Top;
	}
	assert(false);
	return D_Undefined;
}

static vector<CWaypointDistanceMap::CLowResTile> findNeighbors(
	const CWaypointDistanceMap::CLowResTile& lrTile,
	const vector<vector<CWaypointDistanceMap::CLowResTile>>& lrTiles)
{
	int x = lrTile.X;
	int y = lrTile.Y;
	int dx = MaskDX[lrTile.Direction];
	int dy = MaskDY[lrTile.Direction];
	int angle = getRotationAngle(dx, dy);
	simpleRotate(dx, dy, (4 - angle));
	assert(dx == 1 && (dy == 0 || dy == 1));
	vector<CWaypointDistanceMap::CLowResTile> neighbors;
	if (dy == 0) {
		// straight
		int ndx = 1;
		int ndy = 0;
		simpleRotate(ndx, ndy, angle);
		if (checkOpen(lrTile, ndx, ndy, lrTiles)) {
			neighbors.emplace_back(CWaypointDistanceMap::CLowResTile(x + ndx, y + ndy, toDirection(ndx, ndy), lrTiles[x + ndx][y + ndy].GatesMask));
		}
		// turns
		ndx = 1;
		ndy = 1;
		simpleRotate(ndx, ndy, angle);
		if (checkOpen(lrTile, ndx, ndy, lrTiles)) {
			neighbors.emplace_back(CWaypointDistanceMap::CLowResTile(x + ndx, y + ndy, toDirection(ndx, ndy), lrTiles[x + ndx][y + ndy].GatesMask));
		}
		ndx = 1;
		ndy = -1;
		simpleRotate(ndx, ndy, angle);
		if (checkOpen(lrTile, ndx, ndy, lrTiles)) {
			neighbors.emplace_back(CWaypointDistanceMap::CLowResTile(x + ndx, y + ndy, toDirection(ndx, ndy), lrTiles[x + ndx][y + ndy].GatesMask));
		}
	} else if (dy == 1) {
		// diagonal
		int ndx = 1;
		int ndy = 1;
		simpleRotate(ndx, ndy, angle);
		if (checkOpen(lrTile, ndx, ndy, lrTiles)) {
			neighbors.emplace_back(CWaypointDistanceMap::CLowResTile(x + ndx, y + ndy, toDirection(ndx, ndy), lrTiles[x + ndx][y + ndy].GatesMask));
		}
		// turns
		ndx = 1;
		ndy = 0;
		simpleRotate(ndx, ndy, angle);
		if (checkOpen(lrTile, ndx, ndy, lrTiles)) {
			neighbors.emplace_back(CWaypointDistanceMap::CLowResTile(x + ndx, y + ndy, toDirection(ndx, ndy), lrTiles[x + ndx][y + ndy].GatesMask));
		}
		ndx = 0;
		ndy = 1;
		simpleRotate(ndx, ndy, angle);
		if (checkOpen(lrTile, ndx, ndy, lrTiles)) {
			neighbors.emplace_back(CWaypointDistanceMap::CLowResTile(x + ndx, y + ndy, toDirection(ndx, ndy), lrTiles[x + ndx][y + ndy].GatesMask));
		}
	} else {
		assert(false);
	}
	return neighbors;
}

static double smartDistance( const CWaypointDistanceMap::CLowResTile& from,
	const CWaypointDistanceMap::CLowResTile& to)
{
	static double sqrt2 = sqrt(2);
	int dx = to.X - from.X;
	int dy = to.Y - from.Y;
	int angle = getRotationAngle(dx, dy);
	simpleRotate(dx, dy, (4 - angle) % 4);
	assert(dx == 1);
	if (dy == 0) {
		return 1;
	} else {
		assert(dy == 1);
		return sqrt2;
	}
	//assert(false);
	//return -1;
}

CWaypointDistanceMap::CData::CData(int sizeX, int sizeY, const CMyTile& wpTile, const vector<vector<CLowResTile>>& lrTiles)
{
	ClosedSet.assign(sizeX, vector<vector<bool>>(sizeY, vector<bool>(D_EnumSize, false)));
	OpenSet.assign(sizeX, vector<vector<bool>>(sizeY, vector<bool>(D_EnumSize, false)));
	CameFrom.assign(sizeX, vector<vector<CLowResTile>>(sizeY, vector<CLowResTile>(D_EnumSize)));
	Distance.assign(sizeX, vector<vector<double>>(sizeY, vector<double>(D_EnumSize, undefinedDistance)));

	// ���������� ������ ������ � ���������� ����� ������� ������� 0.
	const int startX = wpTile.X * tileSize / step;
	const int endX = (wpTile.X + 1) * tileSize / step;
	const int startY = wpTile.Y * tileSize / step;
	const int endY = (wpTile.Y + 1) * tileSize / step;
	for (int x = startX; x < endX; x++) {
		for (int y = startY; y < endY; y++) {
			for (int d = 0; d < D_EnumSize; d++) {
				Distance[x][y][d] = 0;
				OpenSet[x][y][d] = true;
				Queue.push(CLowResTileWithScore(CLowResTile(x, y, static_cast<TDirection>(d), lrTiles[x][y].GatesMask), 0));
			}
		}
	}

	// ��� ������ ��� ������� (�����. EMPTY) ������� � �������� ���������.
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
			// ������������, ��� ����� ���������� � ��� "�������"
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
	dataByWp.reserve(wpCount);
	for (int i = 0; i < wpCount; i++) {
		dataByWp.emplace_back(CData(sizeX, sizeY, waypoints[i], lowResTiles));
	}
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

static TDirection reverseDirection( TDirection dir )
{
	switch (dir) {
	case D_Right:
		return D_Left;
	case D_RightBot:
		return D_LeftTop;
	case D_Bot:
		return D_Top;
	case D_LeftBot:
		return D_RightTop;
	case D_Left:
		return D_Right;
	case D_LeftTop:
		return D_RightBot;
	case D_Top:
		return D_Bot;
	case D_RightTop:
		return D_LeftBot;
	default:
		assert(false);
		return D_Undefined;
	}
}

double CWaypointDistanceMap::Query(double x, double y, double angle, int waypointIndex)
{
	const int xLowRes = static_cast<int>(x / step);
	const int yLowRes = static_cast<int>(y / step);
	normalizeAngle(angle);
	const TDirection dir = reverseDirection(angleToDirection(angle));
	const double dist = findDistance(xLowRes, yLowRes, dir, dataByWp[waypointIndex]);
	return dist == undefinedDistance ? undefinedDistance : step * dist;
}


double CWaypointDistanceMap::QueryBestDirection(double x, double y, int waypointIndex)
{
	const int xLowRes = static_cast<int>(x / step);
	const int yLowRes = static_cast<int>(y / step);
	double bestDistance = undefinedDistance;
	for (int d = 0; d < D_EnumSize; d++) {
		const double dist = findDistance(xLowRes, yLowRes, static_cast<TDirection>(d), dataByWp[waypointIndex]);
		if (bestDistance == undefinedDistance || (dist != undefinedDistance && dist < bestDistance)) {
			bestDistance = dist;
		}
	}
	//assert(bestDistance != undefinedDistance);
	return bestDistance == undefinedDistance ? undefinedDistance : step * bestDistance;
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
	// ���� �� ��������� ���, �� ���� �� ��� ������. ���� �� ������ ����.
	return undefinedDistance;
}

void CWaypointDistanceMap::processNeighbor(const CLowResTile& from, const CLowResTile& to, CData& data)
{
	if (data.ClosedSet[to.X][to.Y][to.Direction]) {
		return;
	}

	const double dist = data.Distance[from.X][from.Y][from.Direction] + smartDistance(from, to);

	if (!data.OpenSet[to.X][to.Y][to.Direction]) {
		// ������ ��� � openSet - ���������.
		data.Queue.push(CLowResTileWithScore(to, dist));
		data.OpenSet[to.X][to.Y][to.Direction] = true;
	} else if (dist >= data.Distance[to.X][to.Y][to.Direction]) {
		assert(data.Distance[to.X][to.Y][to.Direction] != undefinedDistance);
		// ������� �� ���������� - ����������.
		return;
	} else {
		// ���� � � openSet � ������� ����������. ���� ��������� ������� � ����������� � �������
		// ������� ��������, ����� �������� ��� ���������.
		CPriorityQueue queueCopy;
		CLowResTileWithScore lrTileWithScore = data.Queue.top();
		// ������������ �������� �� ����� ������� � ������, ���� �� ����� ������.
		while (lrTileWithScore.LRTile != to) {
			data.Queue.pop();
			queueCopy.push(lrTileWithScore);
			lrTileWithScore = data.Queue.top();
		}
		// �� ������������ ������ �������.
		data.Queue.pop();
		// ������������ ������� ������� ������� � �������. ���, ����� ������� ��������� �������� ��������
		if (data.Queue.size() < queueCopy.size()) {
			swap(data.Queue, queueCopy);
		}
		while (!queueCopy.empty()) {
			data.Queue.push(queueCopy.top());
			queueCopy.pop();
		}
		// ��������� ���������� ������ �������.
		data.Queue.push(CLowResTileWithScore(to, dist));
	}
	data.CameFrom[to.X][to.Y][to.Direction] = from;
	data.Distance[to.X][to.Y][to.Direction] = dist;
}