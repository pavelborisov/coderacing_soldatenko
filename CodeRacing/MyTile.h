#pragma once

#include <vector>
#include "model\TileType.h"
#include "Arc2D.h"
#include "Vec2D.h"

struct CMyTile {
	int X;
	int Y;
	// ����������� ���� ���� ��������� ����� ����� ������� � CMyTile
	static std::vector<std::vector<model::TileType>> TileTypesXY;
	static double TileSize;
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
	bool IsLeftOpen() const;
	bool IsRightOpen() const;
	bool IsBottomOpen() const;
	bool IsTopOpen() const;
	bool CanDriveTo(const CMyTile& tile) const;
	int Manhattan(const CMyTile& tile) const;
	double Euclidean(const CMyTile& tile) const;
	CVec2D ToVec() const; // ���� ���������� TileSize
	std::vector<CMyTile> FindNeighbors() const;

	static void FillWalls();
	const std::vector<std::pair<CVec2D, CVec2D>>& GetStraightWalls() const;
	const std::vector<CArc2D>& CMyTile::GetArcWalls() const;
	const std::vector<std::pair<CVec2D, double>>& CMyTile::GetCircleWalls() const;

	static int SizeX() { return TileTypesXY.size(); }
	static int SizeY() { return TileTypesXY[0].size(); }

};

inline bool operator < (const CMyTile& a, const CMyTile& b)
{
	if (a.X != b.X) return a.X < b.X;
	return a.Y < b.Y;
}
