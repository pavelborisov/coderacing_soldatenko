#pragma once

struct CVec2D;

#ifdef LOGGING

#include "Debug.h"
class CDrawPlugin {
public:
	static CDrawPlugin& Instance()
	{
		static CDrawPlugin singleInstance;
		return singleInstance;
	}

	void BeginPre();
	void EndPre();
	void BeginPost();
	void EndPost();
	void Circle(double x, double y, double r, int color = 0xFF0000);
	void FillCircle(double x, double y, double r, int color = 0xFF0000);
	void Rect(double x1, double y1, double x2, double y2, int color = 0x00FF00);
	void FillRect(double x1, double y1, double x2, double y2, int color = 0x00FF00);
	void Line(double x1, double y1, double x2, double y2, int color = 0x0000FF);
	void Text(double x, double y, const char* text, int color = 0x000000);

private:
	Debug debug;

	CDrawPlugin();
};

#else // LOGGING is not defined

class CDrawPlugin {
public:
	static CDrawPlugin& Instance()
	{
		static CDrawPlugin singleInstance;
		return singleInstance;
	}

	void BeginPre() {}
	void EndPre() {}
	void BeginPost() {}
	void EndPost() {}
	void Circle(double, double, double, int) {}
	void FillCircle(double, double, double, int) {}
	void Rect(double, double, double, double, int) {}
	void FillRect(double, double, double, double, int) {}
	void Line(double, double, double, double, int) {}
	void Text(double, double, const char*, int) {}

private:

};

#endif

class CDrawPluginSwitcher {
public:
	explicit CDrawPluginSwitcher(CDrawPlugin& _drawPlugin) : drawPlugin(_drawPlugin) { drawPlugin.BeginPost(); }
	~CDrawPluginSwitcher() { drawPlugin.EndPost(); }
	CDrawPlugin& Get() { return drawPlugin; }

private:
	CDrawPlugin& drawPlugin;

	CDrawPluginSwitcher(CDrawPluginSwitcher& switcher) : drawPlugin(switcher.drawPlugin) {}
	CDrawPluginSwitcher& operator = (CDrawPluginSwitcher& /*switcher*/) { return *this; }

};
