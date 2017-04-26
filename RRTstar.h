#pragma once
#include "SMP.h"
#include <list>
class RRTstar : public SMP
{
public:
	virtual void nextIter(std::list<Nodes> &nodes,const std::list<obstacles> obst, Nodes* u_ = NULL);
	std::list<Nodes*> findClosestNeighbours(Nodes n, float radius, std::list<Nodes>& nodes);
};