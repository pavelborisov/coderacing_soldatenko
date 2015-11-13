#pragma once

namespace model {
	class Car;
	class World;
	class Game;
	class Move;
}
struct CMyCar; 
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
	void LogMyCar(const CMyCar& car, const char* name);
	void LogVec2D(const CVec2D& position, const char* name);
	template<typename T>
	void Log(const T& value, const char* name) { logfile << name << ": " << value << endl; }

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
	void LogMyCar(const CMyCar& /*car*/, const char* /*name*/) {}
	void LogVec2D(const CVec2D& /*position*/, const char* /*name*/) {}
	template<typename T>
	void Log(const T& /*value*/, const char* /*name*/) {}

private:
	CLog() {}

};
#endif