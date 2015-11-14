#pragma once

#include <vector>
#include "model\TileType.h"
#include "Vec2D.h"

struct CMyTile {
	int X;
	int Y;
	// Статические поля надо выставить перед любой работой с CMyTile
	static std::vector<std::vector<model::TileType>> TileTypesXY;
	static double TileSize;

	CMyTile();
	CMyTile(int X, int Y);

	bool operator==(const CMyTile& tile) const;
	bool operator!=(const CMyTile& tile) const;

	model::TileType Type() const { return TileTypesXY[X][Y]; }
	bool IsEmpty() const;
	bool IsLeftOpen() const;
	bool IsRightOpen() const;
	bool IsBottomOpen() const;
	bool IsTopOpen() const;
	bool CanDriveTo(const CMyTile& tile) const;
	int Manhattan(const CMyTile& tile) const;
	CVec2D ToVec() const; // Надо выыставить TileSize
	std::vector<CMyTile> FindNeighbors() const;

	static int SizeX() { return TileTypesXY.size(); }
	static int SizeY() { return TileTypesXY[0].size(); }

};
