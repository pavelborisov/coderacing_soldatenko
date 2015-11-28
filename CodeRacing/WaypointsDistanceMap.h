#pragma once

#include <algorithm>
#include <functional>
#include <queue>
#include <vector>
#include "MyTile.h"

class CWaypointDistanceMap {
public:
	static const int tileSize = 800;
	static const int step = 80;
	static const int undefinedCoordinate = -1;
	static const int undefinedDistance = -1;
	static const int undefinedScore = -1;

	struct CLowResTile {
		int X;
		int Y;
		unsigned int NeighborsMask;

		CLowResTile() : X(undefinedCoordinate), Y(undefinedCoordinate), NeighborsMask(0) {}
	};

	struct CLowResTileWithScore {
		CLowResTile LRTile;
		double Score;

		CLowResTileWithScore() : Score(undefinedScore) {}
		CLowResTileWithScore(const CLowResTile& LRTile, double Score) : LRTile(LRTile), Score(Score) {}
	};

	void Initialize(const std::vector<CMyTile>& waypoints);
	double Query(double x, double y, int waypointIndex);

private:
	typedef std::priority_queue<CWaypointDistanceMap::CLowResTileWithScore,
		std::vector<CWaypointDistanceMap::CLowResTileWithScore>,
		std::greater<CWaypointDistanceMap::CLowResTileWithScore >> CPriorityQueue;

	struct CData {
		std::vector<std::vector<bool>> ClosedSet;
		std::vector<std::vector<bool>> OpenSet;
		CPriorityQueue Queue;
		std::vector<std::vector<CLowResTile>> CameFrom;
		//vector<vector<double>> TotalScore;
		std::vector<std::vector<double>> Distance;

		CData(int sizeX, int sizeY, const CMyTile& wpTile, const std::vector<std::vector<CLowResTile>>& lrTiles);
	};

	std::vector<CMyTile> waypoints;
	std::vector<std::vector<CLowResTile>> lowResTiles;
	std::vector<CData> dataByWp;

	double findDistance(int xt, int yt, CData& data);
	void processNeighbor(int x, int y, int xn, int yn, CData& data);

};
