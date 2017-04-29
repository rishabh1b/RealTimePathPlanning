#pragma once
#include "RRTstar.h"
class InformedRRTstar : public RRTstar
{
public:
	void nextIter(std::list<Nodes> &nodes, std::list<obstacles*> obst);
	static Nodes sample(float c_max);
	static bool usingInformedRRTstar;
protected:
	std::list<Nodes*> sol_nodes;
};