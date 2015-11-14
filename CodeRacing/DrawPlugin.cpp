#include "DrawPlugin.h"

#ifdef LOGGING

#include "Vec2D.h"
using namespace std;

CDrawPlugin::CDrawPlugin()
{
	drawFilePath = "..\\draw.txt";
}

CDrawPlugin::~CDrawPlugin()
{
}

void CDrawPlugin::BeginDraw()
{
	drawFile.open(drawFilePath.c_str());
}

void CDrawPlugin::SetColor(int red, int green, int blue)
{
	drawFile << "setColor " << red << " " << green << " " << blue << endl;
}

void CDrawPlugin::DrawLine(const CVec2D& start, const CVec2D& end)
{
	drawFile << "drawLine " << start.X << " " << start.Y << " " << end.X << " " << end.Y << endl;
}

void CDrawPlugin::FillCircle(const CVec2D& position, double radius)
{
	drawFile << "fillCircle " << position.X << " " << position.Y << " " << radius << endl;
}

void CDrawPlugin::EndDraw()
{
	drawFile.close();
}

#endif
