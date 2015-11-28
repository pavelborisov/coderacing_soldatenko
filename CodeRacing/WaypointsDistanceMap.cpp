#include "WaypointsDistanceMap.h"

#include <assert.h>

using namespace std;

const unsigned int Left = 1 << 0;
const unsigned int Right = 1 << 1;
const unsigned int Top = 1 << 2;
const unsigned int Bottom = 1 << 3;

bool operator == (const CWaypointDistanceMap::CLowResTile& a, const CWaypointDistanceMap::CLowResTile& b)
{
	return a.X == b.X && a.Y == b.Y;
}

bool operator != (const CWaypointDistanceMap::CLowResTile& a, const CWaypointDistanceMap::CLowResTile& b)
{
	return a.X != b.X || a.Y != b.Y;
}

bool operator > (const CWaypointDistanceMap::CLowResTileWithScore& a, const CWaypointDistanceMap::CLowResTileWithScore& b)
{
	// TODO: Для стабильности - добавить сравнение по X и Y в случае равенства.
	return a.Score > b.Score;
}

CWaypointDistanceMap::CData::CData(int sizeX, int sizeY, const CMyTile& wpTile, const vector<vector<CLowResTile>>& lrTiles)
{
	ClosedSet.assign(sizeX, vector<bool>(sizeY, false));
	OpenSet.assign(sizeX, vector<bool>(sizeY, false));
	CameFrom.assign(sizeX, vector<CLowResTile>(sizeY));
	//TotalScore.assign(sizeX, vector<double>(sizeY, undefinedScore));
	Distance.assign(sizeX, vector<double>(sizeY, undefinedDistance));

	// Расстояния внутри клетки с вейпоинтом будем считать равными 0.
	const int startX = wpTile.X * tileSize / step;
	const int endX = (wpTile.X + 1) * tileSize / step;
	const int startY = wpTile.Y * tileSize / step;
	const int endY = (wpTile.Y + 1) * tileSize / step;
	for (int x = startX; x < endX; x++) {
		for (int y = startY; y < endY; y++) {
			Distance[x][y] = 0;
			//TotalScore[x][y] = 0;
			ClosedSet[x][y] = true;
			const unsigned int neighborsMask = lrTiles[x][y].NeighborsMask;
			if (x == startX && (neighborsMask & Left) != 0) {
				const int dist = 1;
				Distance[x - 1][y] = dist;
				OpenSet[x - 1][y] = true;
				CameFrom[x - 1][y] = lrTiles[x][y];
				Queue.push(CLowResTileWithScore(lrTiles[x - 1][y], dist));
			}
			if (x == endX - 1 && (neighborsMask & Right) != 0) {
				const int dist = 1;
				Distance[x + 1][y] = dist;
				OpenSet[x + 1][y] = true;
				CameFrom[x + 1][y] = lrTiles[x][y];
				Queue.push(CLowResTileWithScore(lrTiles[x + 1][y], dist));
			}
			if (y == startY && (neighborsMask & Top) != 0) {
				const int dist = 1;
				Distance[x][y - 1] = dist;
				OpenSet[x][y - 1] = true;
				CameFrom[x][y - 1] = lrTiles[x][y];
				Queue.push(CLowResTileWithScore(lrTiles[x][y - 1], dist));
			}
			if (y == endY - 1 && (neighborsMask & Bottom) != 0) {
				const int dist = 1;
				Distance[x][y + 1] = dist;
				OpenSet[x][y + 1] = true;
				CameFrom[x][y + 1] = lrTiles[x][y];
				Queue.push(CLowResTileWithScore(lrTiles[x][y + 1], dist));
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
			const bool leftNeighbor = tileX > 0 && tile.IsLeftOpen();
			const bool rightNeighbor = tileX < tilesSizeX - 1 && tile.IsRightOpen();
			const bool topNeighbor = tileY > 0 && tile.IsTopOpen();
			const bool bottomNeighbor = tileY < tilesSizeY - 1 && tile.IsBottomOpen();

			for (int x = startX; x < endX; x++) {
				for (int y = startY; y < endY; y++) {
					CLowResTile& lrTile = lowResTiles[x][y];
					lrTile.X = x;
					lrTile.Y = y;
					if (tile.Type() == model::TileType::EMPTY) continue;
					if (x > startX || leftNeighbor) {
						lrTile.NeighborsMask |= Left;
					} 
					if (x < endX - 1 || rightNeighbor) {
						lrTile.NeighborsMask |= Right;
					}
					if (y > startY || topNeighbor) {
						lrTile.NeighborsMask |= Top;
					}
					if (y < endY - 1 || bottomNeighbor) {
						lrTile.NeighborsMask |= Bottom;
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

double CWaypointDistanceMap::Query(double x, double y, int waypointIndex)
{
	const int xLowRes = static_cast<int>(x / step);
	const int yLowRes = static_cast<int>(y / step);
	return step * findDistance(xLowRes, yLowRes, dataByWp[waypointIndex]);
}

double CWaypointDistanceMap::findDistance(int xt, int yt, CData& data)
{
	if (data.ClosedSet[xt][yt]) {
		return data.Distance[xt][yt];
	}
	while (!data.Queue.empty()) {
		CLowResTileWithScore lrTileWithScore = data.Queue.top();
		const CLowResTile& lrTile = lrTileWithScore.LRTile;
		const int x = lrTile.X;
		const int y = lrTile.Y;
		const unsigned int neighborsMask = lrTile.NeighborsMask;
		data.Queue.pop();
		data.OpenSet[x][y] = false;
		data.ClosedSet[x][y] = true;

		if ((neighborsMask & Left) != 0) processNeighbor(x, y, x - 1, y, data);
		if ((neighborsMask & Right) != 0) processNeighbor(x, y, x + 1, y, data);
		if ((neighborsMask & Top) != 0) processNeighbor(x, y, x, y - 1, data);
		if ((neighborsMask & Bottom) != 0) processNeighbor(x, y, x, y + 1, data);

		if (x == xt && y == yt) {
			return data.Distance[x][y];
		}
	}
	// Если мы оказались тут, то путь не был найден. Чего не должно быть.
	assert(false);
	return undefinedDistance;
}

void CWaypointDistanceMap::processNeighbor(int x, int y, int xn, int yn, CData& data)
{
	if (data.ClosedSet[xn][yn]) {
		return;
	}

	CLowResTile neighborLrTile = lowResTiles[xn][yn];
	const double dist = data.Distance[x][y] + 1;

	if (!data.OpenSet[xn][yn]) {
		// Соседа нет в openSet - добавляем.
		data.Queue.push(CLowResTileWithScore(neighborLrTile, dist));
		data.OpenSet[xn][yn] = true;
	} else if (dist >= data.Distance[xn][yn]) {
		assert(data.Distance[xn][yn] != undefinedDistance);
		// Функция не улучшилась - пропускаем.
		return;
	} else {
		// Есть и в openSet и функция улучшилась. Надо перебрать очередь с приоритетом в поисках
		// данного элемента, чтобы изменить ему приоритет.
		CPriorityQueue queueCopy;
		CLowResTileWithScore lrTileWithScore = data.Queue.top();
		// Перекидываем элементы из одной очереди в другую, пока не найдём нужный.
		while (lrTileWithScore.LRTile != neighborLrTile) {
			data.Queue.pop();
			queueCopy.push(lrTileWithScore);
			lrTileWithScore = data.Queue.top();
		}
		// Не перекидываем нужный элемент.
		data.Queue.pop();
		// Перекидываем остатки меньшей очереди в большую. Так, чтобы большая оказалась исходной очередью
		if (data.Queue.size() < queueCopy.size()) {
			swap(data.Queue, queueCopy);
		}
		while (!queueCopy.empty()) {
			data.Queue.push(queueCopy.top());
			queueCopy.pop();
		}
		// Добавляем обновлённый нужный элемент.
		data.Queue.push(CLowResTileWithScore(neighborLrTile, dist));
	}
	data.CameFrom[xn][yn] = lowResTiles[x][y];
	data.Distance[xn][yn] = dist;
}
