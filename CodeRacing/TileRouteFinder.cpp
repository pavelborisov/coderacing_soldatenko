#include "TileRouteFinder.h"

#include <algorithm>
#include <functional>
#include <queue>
#include "assert.h"
#include "Tools.h"

using namespace std;
using namespace model;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CTileRouteFinder::CMyTileWithScore {
	CMyTile Tile;
	double Score;

	CMyTileWithScore(const CMyTile& _Tile, double _Score) : Tile(_Tile), Score(_Score) {}
};

bool operator > (const CTileRouteFinder::CMyTileWithScore& a, const CTileRouteFinder::CMyTileWithScore& b)
{
	return a.Score > b.Score;
}

bool operator < (const CTileRouteFinder::CMyTileWithScore& a, const CTileRouteFinder::CMyTileWithScore& b)
{
	return a.Score < b.Score;
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



static const double smartDistance(const CMyTile& from, const CMyTile& to)
{
	static const double noBefore = 1;
	static const double straight = 1.5; //1;
	static const double cornerCut = 1; //1.5;
	static const double farTurn = 2; //2;
	static const double closeTurn = 3; //3;
	static const double uTurn = 5; //4;

	return noBefore;
	from;
	to;

	//int dx = to.X - from.X;
	//int dy = to.Y - from.Y;
	//// Повернём так, чтобы dx == 1, что будет соответствовать направлению "вправо".
	//// Один angle = поворот на 90 градусов против часовой стрелки (если ось Y смотрит вниз).
	//int angle = getRotationAngle(dx, dy);
	//simpleRotate(dx, dy, angle);
	//assert(abs(dx) + abs(dy) == 1);

	////const CMyTile bFrom = cameFrom[from.X][from.Y];
	//if (!bFrom.IsCorrect()) {
	//	return noBefore;
	//}
	//int dxb = from.X - bFrom.X;
	//int dyb = from.Y - bFrom.Y;
	//simpleRotate(dxb, dyb, angle);
	//// Отразим, чтобы dyb == 1, что будет соответствовать повороту "снизу направо"
	//bool mirror = dyb == -1;
	//if (mirror) dyb = -dyb;
	//assert(abs(dxb) + abs(dyb) == 1);

	////const CMyTile bbFrom = cameFrom[bFrom.X][bFrom.Y];
	//if (!bbFrom.IsCorrect()) {
	//	return noBefore;
	//}
	//int dxbb = bFrom.X - bbFrom.X;
	//int dybb = bFrom.Y - bbFrom.Y;
	//simpleRotate(dxbb, dybb, angle);
	//// Мы можем ещё раз попробовать отразить, если предыдущий раз не было поворота, а сейчас есть.
	//mirror = mirror || ((dyb == 0) && (dybb == -1));
	//if (mirror) dybb = -dybb;

	//// Из-за нормализации поворота и отражений у нас могут быть всего пять случаев:
	//// ---- прямая дорога, соответствует dx = 1, dxb = 1, dxbb = 1
	//// 
	//// г--  поворот был давно, соответствует dx = 1, dxb = 1, dybb = 1
	////
	////  г-  поворот был недавно, соответствует dx = 1, dyb = 1, dybb = 1
	////  |
	////
	//// _г-  недавно было два переменных поворота, соответствует dx = 1, dyb = 1, dxbb = 1
	////
	////  г-  недавно было два одинаковых поворота. соответствует dx = 1, dyb = 1, dxbb = -1
	////  L-

	//assert(dx == 1);
	//if (dxb == 1 && dxbb == 1) {
	//	return straight;
	//} else if (dxb == 1 && dybb == 1) {
	//	return farTurn;
	//} else if (dyb == 1 && dybb == 1) {
	//	return closeTurn;
	//} else if (dyb == 1 && dxbb == 1) {
	//	return cornerCut;
	//} else if (dyb == 1 && dxbb == -1) {
	//	return uTurn;
	//}
	//assert(false);
	//return 1;
}

static const double heuristic(const CMyTile& from, const CMyTile& to)
{
	return from.Euclidean(to);
}

vector<CMyTile> CTileRouteFinder::findSingleRoute(const CMyTile& start, const CMyTile& end) const
{
	assert(!start.IsEmpty() && !end.IsEmpty());
	if (start == end) {
		return vector<CMyTile>();
	}

	// Что-то похожее на A* (основано на псевдокоде из википедии)
	const int sizeX = CMyTile::SizeX();
	const int sizeY = CMyTile::SizeY();
	vector<vector<bool>> closedSet(sizeX, vector<bool>(sizeY, false));
	vector<vector<bool>> openSet(sizeX, vector<bool>(sizeY, false));
	typedef priority_queue<CMyTileWithScore, vector<CMyTileWithScore>, greater<CMyTileWithScore>> CPriorityQueue;
	CPriorityQueue openSetQueue;
	vector<vector<CMyTile>> cameFrom(sizeX, vector<CMyTile>(sizeY));
	vector<vector<double>> totalScore(sizeX, vector<double>(sizeY, INT_MAX));
	vector<vector<double>> distance(sizeX, vector<double>(sizeY, INT_MAX));

	//if (prevTile.IsCorrect()) {
	//	closedSet[prevTile.X][prevTile.Y] = true;
	//	if (beforePrevTile.IsCorrect()) {
	//		closedSet[beforePrevTile.X][beforePrevTile.Y] = true;
	//	}
	//}

	distance[start.X][start.Y] = 0;
	totalScore[start.X][start.Y] = distance[start.X][start.Y] + heuristic(start, end);
	openSetQueue.push(CMyTileWithScore(start, totalScore[start.X][start.Y]));
	openSet[start.X][start.Y] = true;
	while (!openSet.empty()) {
		// Берём элемент с наименьшим score и убираем его из openSet, добавляя в closedSet.
		CMyTile current = openSetQueue.top().Tile;
		if (current == end) {
			// Раз в очереди элемент с наименьшим приоритетом оказался конечным элементом - это означает,
			// что кратчайший путь найден. Надо восстановить путь и вернуть его.
			vector<CMyTile> singleRoute;
			for (CMyTile prev = cameFrom[current.X][current.Y]; prev != start; prev = cameFrom[prev.X][prev.Y]) {
				singleRoute.push_back(prev);
			}
			singleRoute.push_back(start);
			std::reverse(singleRoute.begin(), singleRoute.end());
			return singleRoute;
		}
		openSetQueue.pop();
		openSet[current.X][current.Y] = false;
		closedSet[current.X][current.Y] = true;
		// Смотрим всех соседей этого элемента.
		vector<CMyTile> neighbors = current.FindNeighbors();
		for (CMyTile neighbor : neighbors) {
			// Не интересуют те соседи, которые уже попали в closedSet
			if (closedSet[neighbor.X][neighbor.Y]) {
				continue;
			}
			//const CMyTile bFrom = current == start ? prevTile : cameFrom[current.X][current.Y];
			//const CMyTile bbFrom = current == start ? beforePrevTile : ( bFrom == start ? prevTile : cameFrom[bFrom.X][bFrom.Y] );
			const double neighborDistance = distance[current.X][current.Y] + smartDistance(current, neighbor);
			const double neighborScore = neighborDistance + heuristic(neighbor, end);
			if (!openSet[neighbor.X][neighbor.Y]) {
				// Соседа нет в openSet - добавляем.
				openSetQueue.push(CMyTileWithScore(neighbor, neighborScore));
				openSet[neighbor.X][neighbor.Y] = true;
			} else if (neighborScore >= totalScore[neighbor.X][neighbor.Y]) {
				// Функция не улучшилась - пропускаем.
				continue;
			} else {
				// Есть и в openSet и функция улучшилась. Надо перебрать очередь с приоритетом в поисках
				// данного элемента, чтобы изменить ему приоритет.
				CPriorityQueue queueCopy;
				CMyTileWithScore tileWithScore = openSetQueue.top();
				// Перекидываем элементы из одной очереди в другую, пока не найдём нужный.
				while (tileWithScore.Tile != neighbor) {
					openSetQueue.pop();
					queueCopy.push(tileWithScore);
					tileWithScore = openSetQueue.top();
				}
				// Не перекидываем нужный элемент.
				openSetQueue.pop();
				// Перекидываем остатки меньшей очереди в большую. Так, чтобы большая оказалась исходной очередью
				if (openSetQueue.size() < queueCopy.size()) {
					swap(openSetQueue, queueCopy);
				}
				while (!queueCopy.empty()) {
					openSetQueue.push(queueCopy.top());
					queueCopy.pop();
				}
				// Добавляем обновлённый нужный элемент.
				openSetQueue.push(CMyTileWithScore(neighbor, neighborScore));
			}
			// Если мы оказались тут, то надо обновить расстояния, score и путь.
			cameFrom[neighbor.X][neighbor.Y] = current;
			distance[neighbor.X][neighbor.Y] = neighborDistance;
			totalScore[neighbor.X][neighbor.Y] = neighborScore;
		}
	}
	// Если мы оказались тут, то путь не был найден. Чего не должно быть.
	assert(false);
	return vector<CMyTile>();
}
