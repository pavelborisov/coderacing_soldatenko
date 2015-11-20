#pragma once

#include "model\Move.h"

struct CMyMove {
	int Turn = 0;   // 0, 1, -1
	int Brake = 0;  // 0, 1
	int Engine = 1; // 1, -1

	CMyMove() {}
	CMyMove(int Turn, int Brake) : Turn(Turn), Brake(Brake) {}
	CMyMove(int Turn, int Brake, int Engine) : Turn(Turn), Brake(Brake), Engine(Engine) {}

	bool operator == (const CMyMove& m) const { return Turn == m.Turn && Brake == m.Brake && Engine == m.Engine; }
	bool operator != (const CMyMove& m) const { return Turn != m.Turn || Brake != m.Brake || Engine != m.Engine; }

	inline model::Move Convert() const;
};

model::Move CMyMove::Convert() const
{
	model::Move result;
	result.setWheelTurn(Turn);
	result.setBrake(Brake == 1);
	result.setEnginePower(Engine);
	return result;
}
