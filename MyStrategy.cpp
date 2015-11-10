#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>

#include "engine/Vector2D.h"

using namespace model;
using namespace std;
using namespace Engine;

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	double nextWaypointX = (self.getNextWaypointX() + 0.5) * game.getTrackTileSize();
	double nextWaypointY = (self.getNextWaypointY() + 0.5) * game.getTrackTileSize();
	CVector2D nextWaypoint(nextWaypointX, nextWaypointY);

	double cornerTileOffset = 0.25 * game.getTrackTileSize();

	switch (world.getTilesXY()[self.getNextWaypointX()][self.getNextWaypointY()]) {
	case LEFT_TOP_CORNER:
		nextWaypoint.Add(cornerTileOffset, cornerTileOffset);
		break;
	case RIGHT_TOP_CORNER:
		nextWaypoint.Add(-cornerTileOffset, cornerTileOffset);
		break;
	case LEFT_BOTTOM_CORNER:
		nextWaypoint.Add(cornerTileOffset, -cornerTileOffset);
		break;
	case RIGHT_BOTTOM_CORNER:
		nextWaypoint.Add(-cornerTileOffset, -cornerTileOffset);
		break;
	}

	double debugAngleToWaypoint = self.getAngleTo(nextWaypointX, nextWaypointY);
	double debugSpeedModule = hypot(self.getSpeedX(), self.getSpeedY());
	debugAngleToWaypoint;
	debugSpeedModule;

	CVector2D selfVector(self.getX(), self.getY());
	CVector2D selfToWaypointVector(nextWaypoint);
	selfToWaypointVector.Subtract(selfVector);
	CVector2D selfUnitVector(1, 0);
	selfUnitVector.Rotate(self.getAngle());
	double angleToWaypoint = selfUnitVector.GetAngle(selfToWaypointVector);
	CVector2D selfSpeedVector(self.getSpeedX(), self.getSpeedY());
	double speedModule = selfSpeedVector.GetLength();
	
	move.setWheelTurn(angleToWaypoint * 32.0 / PI);
	move.setEnginePower(0.75);

	if (speedModule * speedModule * abs(angleToWaypoint) > 2.5 * 2.5 * PI) {
		move.setBrake(true);
	}
}

MyStrategy::MyStrategy() { }
