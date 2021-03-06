#pragma once

#define PI 3.14159265358979323846

enum TDirection {
	D_Undefined = -1,

	D_Right = 0,
	D_RightBot,
	D_Bot,
	D_LeftBot,
	D_Left,
	D_LeftTop,
	D_Top,
	D_RightTop,

	D_EnumSize
};

inline int getRotationAngle(int dx, int dy)
{
	if (dx == 1 && dy == 0) {
		return 0;
	} else if( dx == 1 && dy == 1 ) {
		return 0;
	} else if (dx == 0 && dy == 1) {
		return 1;
	} else if (dx == -1 && dy == 1) {
		return 1;
	} else if (dx == -1 && dy == 0) {
		return 2;
	} else if (dx == -1 && dy == -1) {
		return 2;
	} else if (dx == 0 && dy == -1) {
		return 3;
	} else if (dx == 1 && dy == -1) {
		return 3;
	}

	//assert(false);
	return -1;
}

template<typename T>
void simpleRotate(T& x, T& y, int angle)
{
	T temp;
	switch (angle) {
		case 0:
		case 4:
			break;
		case 1:
			temp = x;
			x = -y;
			y = temp;
			break;
		case 2:
			x = -x;
			y = -y;
			break;
		case 3:
			temp = x;
			x = y;
			y = -temp;
			break;
		default:
			//assert(false);
			break;
	}
}

inline void normalizeAngle(double& angle)
{
	while (angle > PI) {
		angle -= 2 * PI;
	}
	while (angle < -PI) {
		angle += 2 * PI;
	}
}
