#pragma once

namespace model {
	class Car;
	class World;
	class Game;
	class Move;
}
struct CVec2D;

#ifdef LOGGING

#include <fstream>
class CLog {
public:
	static CLog& Instance()
	{
		static CLog singleInstance;
		return singleInstance;
	}

	void LogTick(int tick);
	void LogCar(const model::Car& car, const char* name);
	void LogPosition(const CVec2D& position, const char* name);

private:
	std::ofstream logfile;

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

	void LogTick(int /*tick*/) {}
	void LogCar(const model::Car& /*car*/, const char* /*name*/) {}
	void LogPosition(const CVec2D& /*position*/, const char* /*name*/) {}

private:
	CLog() {}

};
#endif