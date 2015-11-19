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



static const double smartDistance(const CMyTile& /*from*/, const CMyTile& /*to*/)
{
	return 1;
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

	// ���-�� ������� �� A* (�������� �� ���������� �� ���������)
	const int sizeX = CMyTile::SizeX();
	const int sizeY = CMyTile::SizeY();
	vector<vector<bool>> closedSet(sizeX, vector<bool>(sizeY, false));
	vector<vector<bool>> openSet(sizeX, vector<bool>(sizeY, false));
	typedef priority_queue<CMyTileWithScore, vector<CMyTileWithScore>, greater<CMyTileWithScore>> CPriorityQueue;
	CPriorityQueue openSetQueue;
	vector<vector<CMyTile>> cameFrom(sizeX, vector<CMyTile>(sizeY));
	vector<vector<double>> totalScore(sizeX, vector<double>(sizeY, INT_MAX));
	vector<vector<double>> distance(sizeX, vector<double>(sizeY, INT_MAX));

	distance[start.X][start.Y] = 0;
	totalScore[start.X][start.Y] = distance[start.X][start.Y] + heuristic(start, end);
	openSetQueue.push(CMyTileWithScore(start, totalScore[start.X][start.Y]));
	openSet[start.X][start.Y] = true;
	while (!openSet.empty()) {
		// ���� ������� � ���������� score � ������� ��� �� openSet, �������� � closedSet.
		CMyTile current = openSetQueue.top().Tile;
		if (current == end) {
			// ��� � ������� ������� � ���������� ����������� �������� �������� ��������� - ��� ��������,
			// ��� ���������� ���� ������. ���� ������������ ���� � ������� ���.
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
		// ������� ���� ������� ����� ��������.
		vector<CMyTile> neighbors = current.FindNeighbors();
		for (CMyTile neighbor : neighbors) {
			// �� ���������� �� ������, ������� ��� ������ � closedSet
			if (closedSet[neighbor.X][neighbor.Y]) {
				continue;
			}
			const double neighborDistance = distance[current.X][current.Y] + smartDistance(current, neighbor);
			const double neighborScore = neighborDistance + heuristic(neighbor, end);
			if (!openSet[neighbor.X][neighbor.Y]) {
				// ������ ��� � openSet - ���������.
				openSetQueue.push(CMyTileWithScore(neighbor, neighborScore));
				openSet[neighbor.X][neighbor.Y] = true;
			} else if (neighborScore >= totalScore[neighbor.X][neighbor.Y]) {
				// ������� �� ���������� - ����������.
				continue;
			} else {
				// ���� � � openSet � ������� ����������. ���� ��������� ������� � ����������� � �������
				// ������� ��������, ����� �������� ��� ���������.
				CPriorityQueue queueCopy;
				CMyTileWithScore tileWithScore = openSetQueue.top();
				// ������������ �������� �� ����� ������� � ������, ���� �� ����� ������.
				while (tileWithScore.Tile != neighbor) {
					openSetQueue.pop();
					queueCopy.push(tileWithScore);
					tileWithScore = openSetQueue.top();
				}
				// �� ������������ ������ �������.
				openSetQueue.pop();
				// ������������ ������� ������� ������� � �������. ���, ����� ������� ��������� �������� ��������
				if (openSetQueue.size() < queueCopy.size()) {
					swap(openSetQueue, queueCopy);
				}
				while (!queueCopy.empty()) {
					openSetQueue.push(queueCopy.top());
					queueCopy.pop();
				}
				// ��������� ���������� ������ �������.
				openSetQueue.push(CMyTileWithScore(neighbor, neighborScore));
			}
			// ���� �� ��������� ���, �� ���� �������� ����������, score � ����.
			cameFrom[neighbor.X][neighbor.Y] = current;
			distance[neighbor.X][neighbor.Y] = neighborDistance;
			totalScore[neighbor.X][neighbor.Y] = neighborScore;
		}
	}
	// ���� �� ��������� ���, �� ���� �� ��� ������. ���� �� ������ ����.
	assert(false);
	return vector<CMyTile>();
}
