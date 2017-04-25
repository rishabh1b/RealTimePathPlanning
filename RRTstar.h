#pragma once
#include "SMP.h"
#include <list>
class RRTstar : public SMP
{
public:
	void nextIter(std::list<Nodes> &nodes,const std::list<obstacles> obst);
	std::list<Nodes*> findClosestNeighbours(Nodes n, float radius, std::list<Nodes>& nodes);
};