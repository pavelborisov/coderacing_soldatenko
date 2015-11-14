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

void CDrawPlugin::DrawCircle(const CVec2D& position, double radius)
{
	drawFile << "drawCircle(" << position.X << ", " << position.Y << ", " << radius << ");" << endl;
}

void CDrawPlugin::EndDraw()
{
	drawFile.close();
}

#endif
