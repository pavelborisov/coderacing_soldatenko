#pragma once

#include <algorithm>
#include <functional>
#include <queue>
#include <vector>
#include "MyTile.h"
#include "Tools.h"

class CWaypointDistanceMap {
public:
	static const int tileSize = 800;
	static const int step = 400;
	static const int undefinedCoordinate = -1;
	static const int undefinedDistance = -1;
	static const int undefinedScore = -1;

	///////////////////////
	struct CLowResTile {
		int X;
		int Y;
		TDirection Direction;
		unsigned int GatesMask;

		CLowResTile() : X(undefinedCoordinate), Y(undefinedCoordinate), Direction(D_Undefined), GatesMask(0) {}
		CLowResTile(int X, int Y, TDirection Direction, unsigned int GatesMask) :
			X(X), Y(Y), Direction(Direction), GatesMask(GatesMask) {}
	};

	
	///////////////////////
	struct CLowResTileWithScore {
		CLowResTile LRTile;
		double Score;

		CLowResTileWithScore() : Score(undefinedScore) {}
		CLowResTileWithScore(const CLowResTile& LRTile, double Score) : LRTile(LRTile), Score(Score) {}
	};

	///////////////////////
	static CWaypointDistanceMap& Instance()
	{
		static CWaypointDistanceMap singleInstance;
		return singleInstance;
	}

	void Initialize(const std::vector<CMyTile>& waypoints);
	double Query(double x, double y, double angle, int waypointIndex);
	double QueryBestDirection(double x, double y, int waypointIndex);

private:
	///////////////////////
	typedef std::priority_queue<CWaypointDistanceMap::CLowResTileWithScore,
		std::vector<CWaypointDistanceMap::CLowResTileWithScore>,
		std::greater<CWaypointDistanceMap::CLowResTileWithScore >> CPriorityQueue;

	///////////////////////
	struct CData {
		std::vector<std::vector<std::vector<bool>>> ClosedSet;
		std::vector<std::vector<std::vector<bool>>> OpenSet;
		CPriorityQueue Queue;
		std::vector<std::vector<std::vector<CLowResTile>>> CameFrom;
		std::vector<std::vector<std::vector<double>>> Distance;

		CData(int sizeX, int sizeY, const CMyTile& wpTile, const std::vector<std::vector<CLowResTile>>& lrTiles);
	};

	///////////////////////
	std::vector<CMyTile> waypoints;
	std::vector<std::vector<CLowResTile>> lowResTiles;
	std::vector<CData> dataByWp;

	CWaypointDistanceMap() {}

	double findDistance(int xt, int yt, TDirection dirt, CData& data);
	void processNeighbor(const CLowResTile& from, const CLowResTile& to, CData& data);

};