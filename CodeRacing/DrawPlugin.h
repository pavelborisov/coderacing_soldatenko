#pragma once

struct CVec2D;

#ifdef LOGGING

#include <string>
#include <fstream>

class CDrawPlugin {
public:
	static CDrawPlugin& Instance()
	{
		static CDrawPlugin singleInstance;
		return singleInstance;
	}

	void BeginDraw();
	void DrawCircle(const CVec2D& position, double radius);
	void EndDraw();

private:
	std::string drawFilePath;
	std::ofstream drawFile;

	CDrawPlugin();
	~CDrawPlugin();
};

#else // LOGGING is not defined

class CDrawPlugin {
public:
	static CDrawPlugin& Instance()
	{
		static CDrawPlugin singleInstance;
		return singleInstance;
	}

	void BeginDraw();
	void DrawCircle(const CVec2D& /*position*/, double /*radius*/) {}
	void EndDraw();

private:

};

#endif

class CDrawPluginSwitcher {
public:
	explicit CDrawPluginSwitcher(CDrawPlugin& _drawPlugin) : drawPlugin(_drawPlugin) { drawPlugin.BeginDraw(); }
	~CDrawPluginSwitcher() { drawPlugin.EndDraw(); }
	CDrawPlugin& Get() { return drawPlugin; }

private:
	CDrawPlugin& drawPlugin;

	CDrawPluginSwitcher(CDrawPluginSwitcher& switcher) : drawPlugin(switcher.drawPlugin) {}
	CDrawPluginSwitcher& operator = (CDrawPluginSwitcher& /*switcher*/) {}

};
