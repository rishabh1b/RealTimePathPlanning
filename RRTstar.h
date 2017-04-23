#pragma once
#include "SMP.h"
#include <list>
class RRTstar : public SMP
{
public:
	std::list<Nodes*> findClosestNeighbours(Nodes n, float radius, std::list<Nodes>& nodes);
};