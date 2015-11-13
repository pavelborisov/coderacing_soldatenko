#pragma once

namespace model {
	class Car;
	class World;
	class Game;
	class Move;
}

#ifdef LOGGING

#include <fstream>
class CLog {
public:
	static CLog& Instance()
	{
		static CLog singleInstance;
		return singleInstance;
	}

	void Log(const model::Car& car, const model::World& world, const model::Game& game, model::Move& move);

private:
	std::ofstream logfile;
	int predTick;

	CLog();
	~CLog();
};

#else // LOGGING is not defined

class CLog {
public:
	static CLog& Instance()
	{
		static CLog singleInstance;
		return singleInstance;
	}

	void Log(const model::Car& /*car*/, const model::World& /*world*/, const model::Game& /*game*/, model::Move& /*move*/) {}

private:
	CLog() {}

};
#endif