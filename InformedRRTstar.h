#pragma once
#include "RRTstar.h"
class InformedRRTstar : public RRTstar
{
public:
	Nodes sample(float c_max);
	void nextIter(std::list<Nodes> &nodes, std::list<obstacles*> obst);
private:
	std::list<Nodes*> sol_nodes;
};