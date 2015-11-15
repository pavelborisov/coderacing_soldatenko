#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

#include "DrawPlugin.h"
#include "Log.h"
#include "MyCar.h"
#include "MyTile.h"
#include "Simulator.h"
#include "TileRouteFinder.h"
#include "Vec2D.h"

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

	CTileRouteFinder tileRouteFinder;
	std::vector<CMyTile> waypointTiles;
	CMyTile beforePrevTile;
	CMyTile prevTile;
	CMyTile currentTile;
	std::vector<CMyTile> tileRoute;
	int nextWaypointIndex;

	CSimulator simulator;
	CMyCar prevPrediction;
	CMyCar car;
	CMyCar prediction;

	MyStrategy& operator = (const MyStrategy&) {}

	void updateWaypoints();
	void findTileRoute();
	void firstTick();
	void makeMove();
	void predict();
	void doLog();
	void doDraw();
};

#endif
