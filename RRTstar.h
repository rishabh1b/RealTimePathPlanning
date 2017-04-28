#pragma once
#include "SMP.h"
#include <list>
class RRTstar : public SMP
{
public:
	//virtual void nextIter(std::list<Nodes> &nodes,const std::list<obstacles> obst, Nodes* u_ = NULL);
	virtual void nextIter(Data &tree, const std::list<obstacles> obst, Nodes* u_ = NULL);
	std::list<Nodes*> findClosestNeighbours(Nodes n, float radius, Data *tree);
	//std::list<Nodes*> findClosestNeighbours(Nodes n, float radius, std::list<Nodes>& nodes);
};