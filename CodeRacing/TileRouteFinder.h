#pragma once

#include <vector>
#include "MyTile.h"

class CTileRouteFinder {
public:
	struct CMyTileWithScore;

	std::vector<CMyTile> FindRoute(
		const std::vector<CMyTile>& waypointTiles, int nextWaypointIndex, const CMyTile& currentTile,
		const CMyTile& prevTile, const CMyTile& beforePrevTile ) const;

private:
	std::vector<CMyTile> findSingleRoute( const CMyTile& start, const CMyTile& end,
		const CMyTile& prevTile, const CMyTile& beforePrevTile ) const;

};
