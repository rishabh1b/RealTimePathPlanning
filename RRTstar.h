#pragma once
#include "SMP.h"
#include <list>
class RRTstar : public SMP
{
public:
	virtual void nextIter(std::list<Nodes> &nodes,std::list<obstacles*> obst, Nodes* u_ = NULL);
	static std::list<Nodes*> findClosestNeighbours(Nodes n, std::list<Nodes>& nodes);
};