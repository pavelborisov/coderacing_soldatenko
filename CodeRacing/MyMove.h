#pragma once

#include "model\Move.h"

struct CMyMove {
	double Turn = 0;   // 0, 1, -1
	bool Brake = false;
	double Engine = 1; // 1, -1
	bool Nitro = false;
	bool Shoot = false;
	bool Oil = false;

	CMyMove() {}
	explicit CMyMove(const model::Move& move) :
		Turn(move.getWheelTurn()), Brake(move.isBrake()), Engine(move.getEnginePower()), Nitro(move.isUseNitro()), Shoot(move.isThrowProjectile()), Oil(move.isSpillOil()) {}
	CMyMove(double Turn, bool Brake) : Turn(Turn), Brake(Brake) {}
	CMyMove(double Turn, bool Brake, double Engine) : Turn(Turn), Brake(Brake), Engine(Engine) {}

	bool operator == (const CMyMove& m) const { return Turn == m.Turn && Brake == m.Brake && Engine == m.Engine && Nitro == m.Nitro && Shoot == m.Shoot && Oil == m.Oil; }
	bool operator != (const CMyMove& m) const { return Turn != m.Turn || Brake != m.Brake || Engine != m.Engine || Nitro != m.Nitro || Shoot != m.Shoot || Oil != m.Oil; }

	inline model::Move Convert() const;
};

model::Move CMyMove::Convert() const
{
	model::Move result;
	result.setWheelTurn(Turn);
	result.setBrake(Brake);
	result.setEnginePower(Engine);
	result.setUseNitro(Nitro);
	result.setThrowProjectile(Shoot);
	result.setSpillOil(Oil);
	return result;
}
