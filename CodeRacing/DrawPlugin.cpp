#include "DrawPlugin.h"

#ifdef LOGGING

#include "Vec2D.h"
using namespace std;

CDrawPlugin::CDrawPlugin()
{
}

void CDrawPlugin::BeginPre()
{
	debug.beginPre();
}

void CDrawPlugin::EndPre()
{
	debug.endPre();
}

void CDrawPlugin::BeginPost()
{
	debug.beginPost();
}

void CDrawPlugin::EndPost()
{
	debug.endPost();
}

void CDrawPlugin::Circle(double x, double y, double r, int32_t color)
{
	debug.circle(x, y, r, color);
}

void CDrawPlugin::FillCircle(double x, double y, double r, int32_t color)
{
	debug.fillCircle(x, y, r, color);
}

void CDrawPlugin::Rect(double x1, double y1, double x2, double y2, int32_t color)
{
	debug.rect(x1, y1, x2, y2, color);
}

void CDrawPlugin::FillRect(double x1, double y1, double x2, double y2, int32_t color)
{
	debug.fillRect(x1, y1, x2, y2, color);
}

void CDrawPlugin::Line(double x1, double y1, double x2, double y2, int32_t color)
{
	debug.line(x1, y1, x2, y2, color);
}

void CDrawPlugin::Text(double x, double y, const char* text, int32_t color)
{
	debug.text(x, y, text, color);
}

#endif
