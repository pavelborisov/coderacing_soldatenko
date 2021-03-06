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
	void Log(const T& value, const char* name) { logfile << name << ": " << value << std::endl; }
	template<typename T>
	void LogIfDifferent(const T& a, const T& b, const char* name)
	{
		if (abs(a - b) > 1e-5) {
			CLog::Instance().Stream() << "Warning! Different " << name << ": "
				<< a << " " << b << " " << a - b << std::endl;
		}
	}
	std::basic_ostream< char, std::char_traits<char> >& Stream() { return logfile; }

private:
	std::ofstream logfile;

	CLog();
	~CLog();
};

#else // LOGGING is not defined

#include <ostream>
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
	template<typename T>
	void LogIfDifferent(const T& /*a*/, const T& /*b*/, const char* /*name*/) {}
	std::basic_ostream< char, std::char_traits<char> >& Stream() { return logfile; }

private:
	std::ostream logfile;
	CLog() : logfile(0) {}

};
#endif