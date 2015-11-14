#include "TileRouteFinder.h"

#include <functional>
#include <queue>
#include "assert.h"

using namespace std;
using namespace model;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CTileRouteFinder::CMyTileWithCost {
	CMyTile Tile;
	int Cost;

	CMyTileWithCost(const CMyTile& _Tile, int _Cost) : Tile(_Tile), Cost(_Cost) {}
};

bool operator > (const CTileRouteFinder::CMyTileWithCost& a, const CTileRouteFinder::CMyTileWithCost& b)
{
	return a.Cost > b.Cost;
}

bool operator < (const CTileRouteFinder::CMyTileWithCost& a, const CTileRouteFinder::CMyTileWithCost& b)
{
	return a.Cost < b.Cost;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

vector<CMyTile> CTileRouteFinder::FindRoute(
	const vector<CMyTile>& waypointTiles, int nextWaypointIndex, const CMyTile& currentTile) const
{
	vector<CMyTile> route;
	CMyTile start = currentTile;
	for (size_t i = 0; i < waypointTiles.size(); i++) {
		CMyTile end = waypointTiles[(nextWaypointIndex + i) % waypointTiles.size()];
		vector<CMyTile> partialRoute = findSingleRoute(start, end);
		route.insert(route.end(), partialRoute.begin(), partialRoute.end());
		start = end;
	}
	vector<CMyTile> finalPartialRoute = findSingleRoute(start, currentTile);
	route.insert(route.end(), finalPartialRoute.begin(), finalPartialRoute.end());
	return route;
}

vector<CMyTile> CTileRouteFinder::findSingleRoute(const CMyTile& start, const CMyTile& end) const
{
	assert(!start.IsEmpty() && !end.IsEmpty());
	if (start == end) {
		return vector<CMyTile>();
	}

	// Что-то похожее на A*
	const int sizeX = CMyTile::SizeX();
	const int sizeY = CMyTile::SizeY();
	priority_queue<CMyTileWithCost, vector<CMyTileWithCost>, greater<CMyTileWithCost>> priority;
	vector<vector<int>> costs(sizeX, vector<int>(sizeY, INT_MAX));
	vector<vector<CMyTile>> previous(sizeX, vector<CMyTile>(sizeY));

	int startCost = 0 + start.Manhattan(end);
	costs[start.X][start.Y] = startCost;
	priority.push(CMyTileWithCost(start, startCost));
	while (!priority.empty()) {
		CMyTile tile = priority.top().Tile;
		const int cost = priority.top().Cost;
		priority.pop();
		// Мы не удаляем из очереди элементы, для которых нашли лучший путь.
		if (cost <= costs[tile.X][tile.Y]) {
			vector<CMyTile> neighbors = tile.FindNeighbors();
			for (CMyTile next : neighbors) {
				// Добавляем всех соседей, которые улучшили своё расстояние, в очередь,
				// запоминая эти расстояния и предков.
				const int dist = 1;
				const int prevDistSum = cost - tile.Manhattan(end);
				const int nextCost = prevDistSum + dist + next.Manhattan(end);
				if (nextCost < costs[next.X][next.Y]) {
					costs[next.X][next.Y] = nextCost;
					previous[next.X][next.Y] = tile;
					priority.push(CMyTileWithCost(next, nextCost));
					// Если достигли конечной точки - собираем маршрут и возвращаемся.
					if (next == end) {
						// Находим путь, разворачиваем его и добавляем. Все тайлы, кроме последнего.
						vector<CMyTile> singleRoute;
						for (CMyTile prev = previous[next.X][next.Y]; prev != start; prev = previous[prev.X][prev.Y]) {
							singleRoute.push_back(prev);
						}
						singleRoute.push_back(start);
						reverse(singleRoute.begin(), singleRoute.end());
						return singleRoute;
					}
				}
			}
		}
	}
	// По идее, мы всегда должны находить путь.
	assert(false);
	return vector<CMyTile>();
}
