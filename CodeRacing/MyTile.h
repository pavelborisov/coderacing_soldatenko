#pragma once

#include <vector>
#include "model\TileType.h"
#include "Arc2D.h"
#include "Vec2D.h"

struct CMyTile {
	int X;
	int Y;
	// Статические поля надо выставить перед любой работой с CMyTile
	static std::vector<std::vector<model::TileType>> TileTypesXY;
	static double TileSize;
	static double WallRadius;
	static std::vector<std::vector<std::vector<std::pair<CVec2D, CVec2D>>>> StraightWallsXY;
	static std::vector<std::vector<std::vector<CArc2D>>> ArcWallsXY;
	static std::vector<std::vector<std::vector<std::pair<CVec2D, double>>>> CircleWallsXY;

	CMyTile();
	CMyTile(int X, int Y);
	explicit CMyTile(const CVec2D& vec);

	bool operator==(const CMyTile& tile) const;
	bool operator!=(const CMyTile& tile) const;

	model::TileType Type() const { return TileTypesXY[X][Y]; }
	bool IsCorrect() const;
	bool IsEmpty() const;
	static bool IsLeftOpen(int x, int y);
	bool IsLeftOpen() const { return IsLeftOpen(X, Y); }
	static bool IsRightOpen(int x, int y);
	bool IsRightOpen() const { return IsRightOpen(X, Y); }
	static bool IsBottomOpen(int x, int y);
	bool IsBottomOpen() const { return IsBottomOpen(X, Y); }
	static bool IsTopOpen(int x, int y);
	bool IsTopOpen() const { return IsTopOpen(X, Y); }
	bool CanDriveTo(const CMyTile& tile) const;
	int Manhattan(const CMyTile& tile) const;
	double Euclidean(const CMyTile& tile) const;
	CVec2D ToVec() const; // Надо выыставить TileSize
	std::vector<CMyTile> FindNeighbors() const;

	static void FillWalls();
	const std::vector<std::pair<CVec2D, CVec2D>>& GetStraightWalls() const;
	const std::vector<CArc2D>& GetArcWalls() const;
	const std::vector<std::pair<CVec2D, double>>& GetCircleWalls() const;

	static int SizeX() { return TileTypesXY.size(); }
	static int SizeY() { return TileTypesXY[0].size(); }

};

inline bool operator < (const CMyTile& a, const CMyTile& b)
{
	if (a.X != b.X) return a.X < b.X;
	return a.Y < b.Y;
}
