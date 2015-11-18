#pragma once

#define PI 3.14159265358979323846
#define PI_2 PI / 2

inline int getRotationAngle(int dx, int dy)
{
	if (dx == 1) {
		return 0;
	} else if (dx == -1) {
		return 2;
	} else if (dy == 1) {
		return 3;
	} else if (dy == -1) {
		return 1;
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
