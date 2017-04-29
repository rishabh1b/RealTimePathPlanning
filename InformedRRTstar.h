#pragma once
#include "RRTstar.h"
class InformedRRTstar : public RRTstar
{
public:
	static Nodes sample(float c_max);
	void nextIter(std::list<Nodes> &nodes, const std::list<obstacles> obst);
	static bool usingInformedRRTstar;
protected:
	std::list<Nodes*> sol_nodes;
};