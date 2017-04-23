#include "SMP.h"

SMP::SMP()
{

}

void SMP::addNode(Nodes n)
{
	nodes.push_back(n);
}

Nodes* SMP::nearestNode(Nodes n)
{
	double min_dist = n.location.squareDistance(nodes.front().location);
	Nodes* near_node = &(nodes.front());
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		it++;
		if (n.location.squareDistance((*it).location) < min_dist)
		{
			min_dist = n.location.squareDistance((*it).location);
			near_node = &(*it);
		}
	}
	return near_node;
}

bool SMP::checkCollision(Nodes n1, Nodes n2)
{
   //TODO: Add Line algorithm
	return true;
}

Nodes SMP::sampler()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	Nodes new_node;
	new_node.location.x = x;
	new_node.location.y = y;
	return new_node;
}

bool SMP::checkSample(Nodes n)
{
	//TODO: check whether samples incorrectly in obstacle region
	return true;
}