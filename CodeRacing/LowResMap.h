#pragma once

#include <vector>
#include "Vec2D.h"

template<class DATA>
class CLowResMap {
public:
	CLowResMap(double width, double height, double size);

	int GetDataWidth() const { return dataWidth; }
	int GetDataHeight() const { return dataHeight; }
	int GetSize() const { return size; }
	const DATA& Get(double x, double y) const;
	void SetPoint(double x, double y, const DATA& data);
	void SetRect(double x1, double y1, double x2, double y2, const DATA& data);
	void SetCircle(double x, double y, double r, const DATA& data);
	void SetRotatedRect(double x, double y, double width, double height, double angle, const DATA& data);

private:
	const int dataWidth;
	const int dataHeight;
	const int size;
	std::vector<std::vector<DATA>> dataXY;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<class DATA>
inline CLowResMap<DATA>::CLowResMap(double width, double height, double size) :
	dataWidth(static_cast<int>(width / size)),
	dataHeight(static_cast<int>(height / size)),
	size(static_cast<int>(size))
{
	dataXY.assign(dataWidth, std::vector<DATA>(dataHeight, DATA()));
}

template<class DATA>
inline const DATA& CLowResMap<DATA>::Get(double x, double y) const
{
	return dataXY[static_cast<int>(x / size)][static_cast<int>(y / size)];
}

template<class DATA>
inline void CLowResMap<DATA>::SetPoint(double x, double y, const DATA& data)
{
	dataXY[static_cast<int>(x / size)][static_cast<int>(y / size)] = data;
}

template<class DATA>
inline void CLowResMap<DATA>::SetRect(double x1, double y1, double x2, double y2, const DATA& data)
{
	const int xmin = static_cast<int>(min(x1, x2) / size);
	const int xmax = static_cast<int>(max(x1, x2) / size);
	const int ymin = static_cast<int>(min(y1, y2) / size);
	const int ymax = static_cast<int>(max(y1, y2) / size);
	for (int x = xmin; x <= xmax; x++) {
		for (int y = ymin; y <= ymax; y++) {
			dataXY[x][y] = data;
		}
	}
}

template<class DATA>
inline void CLowResMap<DATA>::SetCircle(double x, double y, double r, const DATA& data)
{
	const int xi = static_cast<int>(x / size);
	const int yi = static_cast<int>(y / size);
	dataXY[xi][yi] = data;
	const int xmin = static_cast<int>((x - r) / size); // ������, � ������� ������ ����� ����� ����� �����.
	const int xmax = static_cast<int>((x + r) / size); // ������, � ������� ������ ����� ������ ����� �����.
	const int ymin = static_cast<int>((y - r) / size); // ������, � ������� ������ ����� ������� ����� �����.
	const int ymax = static_cast<int>((y + r) / size); // ������, � ������� ������ ����� ������ ����� �����.

	// ����� �����.
	for (int xx = xmin; xx <= xi - 1; xx++) {
		// ���������� �� ������ ���������� �� �����, �����. ������ ���������� ������ xx.
		double xd = x - (xx + 1) * size;
		if (xd > r) continue;
		const double halfHorde = sqrt(r * r - xd * xd);
		const int yTop = static_cast<int>((y - halfHorde) / size);
		const int yBot = static_cast<int>((y + halfHorde) / size);
		for (int yy = yTop; yy <= yBot; yy++) {
			// ������ ���������� ����� � ��������� xx ���� ���������� - ������ ���� �������� ���� ����� ����� � ������ �� ����� �����������
			dataXY[xx][yy] = data;
			dataXY[xx + 1][yy] = data;
		}
	}
	// ������ �����.
	for (int xx = xi + 1; xx <= xmax; xx++) {
		// ���������� �� ������ ���������� �� �����, �����. ����� ���������� ������ xx.
		double xd = xx * size - x;
		if (xd > r) continue;
		const double halfHorde = sqrt(r * r - xd * xd);
		const int yTop = static_cast<int>((y - halfHorde) / size);
		const int yBot = static_cast<int>((y + halfHorde) / size);
		for (int yy = yTop; yy <= yBot; yy++) {
			// ����� ���������� ����� � ��������� xx ���� ���������� - ������ ���� �������� ���� ����� ����� � ������ �� ����� �����������
			dataXY[xx][yy] = data;
			dataXY[xx - 1][yy] = data;
		}
	}
	// ������� �����.
	for (int yy = ymin; yy <= yi - 1; yy++) {
		// ���������� �� ������ ���������� �� �����, �����. ������ ���������� ������ yy.
		double yd = y - (yy + 1) * size;
		if (yd > r) continue;
		const double halfHorde = sqrt(r * r - yd * yd);
		const int xLeft = static_cast<int>((x - halfHorde) / size);
		const int xRight = static_cast<int>((x + halfHorde) / size);
		for (int xx = xLeft; xx <= xRight; xx++) {
			// ������ ���������� ����� � yy ���� ���������� - ������ ���� �������� ���� ����� ������ � ����� �� ����� �����������
			dataXY[xx][yy] = data;
			dataXY[xx][yy + 1] = data;
		}
	}
	// ������ �����.
	for (int yy = yi + 1; yy <= ymax; yy++) {
		// ���������� �� ������ ���������� �� �����, �����. ������ ���������� ������ xx.
		double yd = yy * size - y;
		if (yd > r) continue;
		const double halfHorde = sqrt(r * r - yd * yd);
		const int xLeft = static_cast<int>((x - halfHorde) / size);
		const int xRight = static_cast<int>((x + halfHorde) / size);
		for (int xx = xLeft; xx <= xRight; xx++) {
			// ������� ���������� ����� � yy ���� ���������� - ������ ���� �������� ���� ����� ������ � ����� �� ����� �����������
			dataXY[xx][yy] = data;
			dataXY[xx][yy - 1] = data;
		}
	}
}

template<class DATA>
void CLowResMap<DATA>::SetRotatedRect(double x, double y, double width, double height, double angle, const DATA& data)
{
	const double halfWidth = width / 2;
	const double halfHeight = height / 2;
	CVec2D center(x, y);
	CVec2D corners[] = {
		{ halfWidth, halfHeight },
		{ -halfWidth, halfHeight },
		{ -halfWidth, -halfHeight },
		{ halfWidth, -halfHeight } };
	for (auto& corner : carCorners) {
		corner.Rotate(angle);
		corner += center;
	}
	//TODO
	x;
	y;
	angle;
	data;
}
