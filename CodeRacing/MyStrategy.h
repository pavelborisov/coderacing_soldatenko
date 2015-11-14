#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

#include "Simulator.h"
#include "MyCar.h"
#include "Vec2D.h"
#include "Log.h"
#include "DrawPlugin.h"

class MyStrategy : public Strategy {
public:
	MyStrategy();

	void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& resultMove);

private:
	int currentWaypointIndex;
	CLog& log;
	CDrawPlugin& draw;

	const model::Car* self;
	const model::World* world;
	const model::Game* game;
	model::Move* resultMove;
	int currentTick;

	CSimulator simulator;
	CMyCar prevPrediction;
	CMyCar car;
	CMyCar prediction;

	MyStrategy& operator = (const MyStrategy&) {}

	void firstTick();
	void makeMove();
	void predict();
	void doLog();
	void doDraw();
};

#endif
