#include "MyTile.h"

#include <vector>
#include <cstdlib>
#include <math.h>
#include "Tools.h"

using namespace std;
using namespace model;

vector<vector<TileType>> CMyTile::TileTypesXY;
double CMyTile::TileSize = 800;
double CMyTile::WallRadius = 80;
vector<vector<vector<pair<CVec2D, CVec2D>>>> CMyTile::StraightWallsXY;
vector<vector<vector<CArc2D>>> CMyTile::ArcWallsXY;
vector<vector<vector<pair<CVec2D, double>>>> CMyTile::CircleWallsXY;

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

bool CMyTile::IsLeftOpen(int x, int y)
{
	switch (TileTypesXY[x][y]) {
		case HORIZONTAL:
		case RIGHT_TOP_CORNER:
		case RIGHT_BOTTOM_CORNER:
		case LEFT_HEADED_T:
		case TOP_HEADED_T:
		case BOTTOM_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return x > 0;
		default:
			return false;
	}
}

bool CMyTile::IsRightOpen(int x, int y)
{
	switch (TileTypesXY[x][y]) {
		case HORIZONTAL:
		case LEFT_TOP_CORNER:
		case LEFT_BOTTOM_CORNER:
		case RIGHT_HEADED_T:
		case TOP_HEADED_T:
		case BOTTOM_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return x < SizeX() - 1;
		default:
			return false;
	}
}

bool CMyTile::IsBottomOpen(int x, int y)
{
	switch (TileTypesXY[x][y]) {
		case VERTICAL:
		case LEFT_TOP_CORNER:
		case RIGHT_TOP_CORNER:
		case BOTTOM_HEADED_T:
		case LEFT_HEADED_T:
		case RIGHT_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return y < SizeY() - 1;
		default:
			return false;
	}
}

bool CMyTile::IsTopOpen(int x, int y)
{
	switch (TileTypesXY[x][y]) {
		case VERTICAL:
		case LEFT_BOTTOM_CORNER:
		case RIGHT_BOTTOM_CORNER:
		case TOP_HEADED_T:
		case LEFT_HEADED_T:
		case RIGHT_HEADED_T:
		case CROSSROADS:
		case UNKNOWN:
			return y > 0;
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

void CMyTile::FillWalls()
{
	StraightWallsXY.clear();
	StraightWallsXY.assign(SizeX(), vector<vector<pair<CVec2D, CVec2D>>>(SizeY()));
	ArcWallsXY.clear();
	ArcWallsXY.assign(SizeX(), vector<vector<CArc2D>>(SizeY()));
	CircleWallsXY.clear();
	CircleWallsXY.assign(SizeX(), vector<vector<pair<CVec2D, double>>>(SizeY()));
	for (int x = 0; x < SizeX(); x++) {
		for (int y = 0; y < SizeY(); y++) {
			const CMyTile tile = CMyTile(x, y);
			if (tile.Type() == EMPTY) continue;
			auto& straightWalls = StraightWallsXY[x][y];
			if (!tile.IsLeftOpen()) {
				const double startY = y * TileSize;
				const double endY = (y + 1) * TileSize;
				straightWalls.push_back(make_pair(
					CVec2D(x * TileSize + WallRadius, startY),
					CVec2D(x * TileSize + WallRadius, endY)
					));
			}
			if (!tile.IsRightOpen()) {
				const double startY = (y + 1) * TileSize;
				const double endY = (y) * TileSize;
				straightWalls.push_back(make_pair(
					CVec2D((x + 1) * TileSize - WallRadius, startY),
					CVec2D((x + 1) * TileSize - WallRadius, endY)
					));
			}
			if (!tile.IsTopOpen()) {
				const double startX = (x + 1) * TileSize;
				const double endX = x * TileSize;
				straightWalls.push_back(make_pair(
					CVec2D(startX, y * TileSize + WallRadius),
					CVec2D(endX, y * TileSize + WallRadius)
					));
			}
			if (!tile.IsBottomOpen()) {
				const double startX = x * TileSize;
				const double endX = (x + 1) * TileSize;
				straightWalls.push_back(make_pair(
					CVec2D(startX, (y + 1) * TileSize - WallRadius),
					CVec2D(endX, (y + 1) * TileSize - WallRadius)
					));
			}
			auto& arcWalls = ArcWallsXY[x][y];
			if (!tile.IsLeftOpen() && !tile.IsTopOpen()) {
				arcWalls.push_back(CArc2D(
					CVec2D(x * TileSize + 2 * WallRadius, y * TileSize + 2 * WallRadius),
					WallRadius, -PI, -PI / 2 ));
			}
			if (!tile.IsTopOpen() && !tile.IsRightOpen()) {
				arcWalls.push_back(CArc2D(
					CVec2D((x + 1) * TileSize - 2 * WallRadius, y * TileSize + 2 * WallRadius),
					WallRadius, -PI / 2, 0));
			}
			if (!tile.IsRightOpen() && !tile.IsBottomOpen()) {
				arcWalls.push_back(CArc2D(
					CVec2D((x + 1) * TileSize - 2 * WallRadius, (y + 1) * TileSize - 2 * WallRadius),
					WallRadius, 0, PI / 2));
			}
			if (!tile.IsBottomOpen() && !tile.IsLeftOpen()) {
				arcWalls.push_back(CArc2D(
					CVec2D(x * TileSize + 2 * WallRadius, (y + 1) * TileSize - 2 * WallRadius),
					WallRadius, PI / 2, PI));
			}

			auto& circleWalls = CircleWallsXY[x][y];
			if (tile.IsLeftOpen() && tile.IsTopOpen()) {
				//arcWalls.push_back(CArc2D(
				//	CVec2D(x * TileSize, y * TileSize),
				//	wallRadius, 0, PI / 2));
				circleWalls.push_back(make_pair(CVec2D(x * TileSize, y * TileSize), WallRadius));
			}
			if (tile.IsTopOpen() && tile.IsRightOpen()) {
				//arcWalls.push_back(CArc2D(
				//	CVec2D((x + 1) * TileSize, y * TileSize),
				//	wallRadius, PI / 2, PI));
				circleWalls.push_back(make_pair(CVec2D((x + 1) * TileSize, y * TileSize), WallRadius));
			}
			if (tile.IsRightOpen() && tile.IsBottomOpen()) {
				//arcWalls.push_back(CArc2D(
				//	CVec2D((x + 1) * TileSize, (y + 1) * TileSize),
				//	wallRadius, -PI, -PI / 2));
				circleWalls.push_back(make_pair(CVec2D((x + 1) * TileSize, (y + 1) * TileSize), WallRadius));
			}
			if (tile.IsBottomOpen() && tile.IsLeftOpen()) {
				//arcWalls.push_back(CArc2D(
				//	CVec2D(x * TileSize, (y + 1) * TileSize),
				//	wallRadius, -PI / 2, 0));
				circleWalls.push_back(make_pair(CVec2D(x * TileSize, (y + 1) * TileSize), WallRadius));
			}
		}
	}
}

const vector<pair<CVec2D, CVec2D>>& CMyTile::GetStraightWalls() const
{
	return StraightWallsXY[X][Y];
}
const vector<CArc2D>& CMyTile::GetArcWalls() const
{
	return ArcWallsXY[X][Y];
}
const vector<pair<CVec2D, double>>& CMyTile::GetCircleWalls() const
{
	return CircleWallsXY[X][Y];
}
