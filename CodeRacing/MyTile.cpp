#include "MyTile.h"

#include <vector>
#include <cstdlib>
#include <math.h>

using namespace std;
using namespace model;

vector<vector<TileType>> CMyTile::TileTypesXY = vector<vector<TileType>>();
double CMyTile::TileSize = 800;

CMyTile::CMyTile() : X(-1), Y(-1)
{
}

CMyTile::CMyTile(int _X, int _Y) : X(_X), Y(_Y)
{
}

CMyTile::CMyTile(const CVec2D& vec)
{
	X = static_cast<int>(vec.X / TileSize);
	Y = static_cast<int>(vec.Y / TileSize);
}

bool CMyTile::operator==(const CMyTile& tile) const
{
	return X == tile.X && Y == tile.Y;
}

bool CMyTile::operator!=(const CMyTile& tile) const
{
	return X != tile.X || Y != tile.Y;
}

bool CMyTile::IsCorrect() const
{
	return X >= 0 && X < static_cast<int>(TileTypesXY.size()) && Y >= 0 && Y < static_cast<int>(TileTypesXY[0].size());
}

bool CMyTile::IsEmpty() const
{
	static_assert(_TILE_TYPE_COUNT_ == 13, "model::TileType has been changed");
	switch (Type()) {
		case EMPTY:
		case _UNKNOWN_TILE_TYPE_:
			return true;
		default:
			return false;
	}
}

bool CMyTile::IsLeftOpen() const
{
	switch (Type()) {
		case HORIZONTAL:
		case RIGHT_TOP_CORNER:
		case RIGHT_BOTTOM_CORNER:
		case LEFT_HEADED_T:
		case TOP_HEADED_T:
		case BOTTOM_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return true;
		default:
			return false;
	}
}

bool CMyTile::IsRightOpen() const
{
	switch (Type()) {
		case HORIZONTAL:
		case LEFT_TOP_CORNER:
		case LEFT_BOTTOM_CORNER:
		case RIGHT_HEADED_T:
		case TOP_HEADED_T:
		case BOTTOM_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return true;
		default:
			return false;
	}
}

bool CMyTile::IsBottomOpen() const
{
	switch (Type()) {
		case VERTICAL:
		case LEFT_TOP_CORNER:
		case RIGHT_TOP_CORNER:
		case BOTTOM_HEADED_T:
		case LEFT_HEADED_T:
		case RIGHT_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return true;
		default:
			return false;
	}
}

bool CMyTile::IsTopOpen() const
{
	switch (Type()) {
		case VERTICAL:
		case LEFT_BOTTOM_CORNER:
		case RIGHT_BOTTOM_CORNER:
		case TOP_HEADED_T:
		case LEFT_HEADED_T:
		case RIGHT_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return true;
		default:
			return false;
	}
}

bool CMyTile::CanDriveTo(const CMyTile& tile) const
{
	const int dx = tile.X - X;
	const int dy = tile.Y - Y;
	if (dx == 1) {
		return IsRightOpen() && tile.IsLeftOpen();
	} else if (dx == -1) {
		return IsLeftOpen() && tile.IsRightOpen();
	} else if (dy == 1) {
		return IsBottomOpen() && tile.IsTopOpen();
	} else if (dy == -1) {
		return IsTopOpen() && tile.IsBottomOpen();
	}
	return false;
}

int CMyTile::Manhattan(const CMyTile& tile) const
{
	return abs(X - tile.X) + abs(Y - tile.Y);
}

double CMyTile::Euclidean(const CMyTile& tile) const
{
	return sqrt(pow(X - tile.X, 2) + pow(Y - tile.Y, 2));
}

CVec2D CMyTile::ToVec() const
{
	return CVec2D((X + 0.5) * TileSize, (Y + 0.5) * TileSize);
}

vector<CMyTile> CMyTile::FindNeighbors() const
{
	static const int dxyCount = 4;
	static const int dxData[] = { 1, 0, -1, 0 };
	static const int dyData[] = { 0, 1, 0, -1 };

	vector<CMyTile> neighbors;
	for (int i = 0; i < dxyCount; i++) {
		const int dx = dxData[i];
		const int dy = dyData[i];
		CMyTile tile(X + dx, Y + dy);
		if (tile.X >= 0 && tile.X < SizeX() && tile.Y >= 0 && tile.Y < SizeY()
			&& CanDriveTo(tile))
		{
			neighbors.push_back(tile);
		}
	}
	return neighbors;
}