#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

#include "BestMoveFinder.h"
#include "DrawPlugin.h"
#include "Log.h"
#include "MyCar.h"
#include "MyTile.h"
#include "MyWorld.h"
#include "Vec2D.h"
#include "WaypointsDistanceMap.h"

class MyStrategy : public Strategy {
public:
	MyStrategy();

	void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& resultMove);

private:
	CLog& log;
	CDrawPlugin& draw;

	const model::Car* self;
	const model::World* world;
	const model::Game* game;
	model::Move* resultMove;
	int currentTick;
	static long long randomSeed;

	std::vector<CMyTile> waypointTiles;
	CMyTile currentTile;
	int nextWaypointIndex;

	CMyWorld previousPredictedWorld;
	CMyWorld currentWorld;
	CMyCar currentCar;
	CMyWorld predictedWorld;;
	CBestMoveFinder::CResult previousResult;
	static CBestMoveFinder::CResult allyResult[2];
	static int allyResultTick[2];
	
	int rear;
	CVec2D stoppedPosition;
	int stoppedTicks;
	static const double stoppedLengthThreshold;
	static const int stoppedTicksThreshold;

	MyStrategy& operator = (const MyStrategy&) { return *this; }

	void updateWaypoints();
	void makeMove();
	void predictObjects();
	void processShooting();
	void processOil();
	void experiment();
	void predict();
	void doLog();
	void doDraw();
	void saveMap();
};

#endif
