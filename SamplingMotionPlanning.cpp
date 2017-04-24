#include "SMP.h"
#include "RRTstar.h"

SMP::SMP()
{

}

void SMP::addNode(Nodes n, std::list<Nodes>& nodes)
{
	nodes.push_back(n);
}

Nodes* SMP::nearestNode(Nodes n, std::list<Nodes>& nodes)
{
	double min_dist = n.location.squareDistance(nodes.front().location);
	Nodes* near_node = &(nodes.front());
	if (!nodes.empty())
	{
		std::list<Nodes>::iterator it = nodes.begin();
		while (it != nodes.end())
		{
			if (n.location.squareDistance((*it).location) < min_dist)
			{
				min_dist = n.location.squareDistance((*it).location);
				near_node = &(*it);
			}
			it++;
		}
		return near_node;
	}
	else
		return NULL;
}

Nodes* SMP::nearestNode(Nodes n, std::list<Nodes*>& nodes)
{
	double min_dist = n.location.squareDistance(nodes.front()->location);
	Nodes* near_node = nodes.front();
	if (!nodes.empty())
	{
		std::list<Nodes*>::iterator it = nodes.begin();
		while (it != nodes.end())
		{
			if (n.location.squareDistance((*it)->location) < min_dist)
			{
				min_dist = n.location.squareDistance((*it)->location);
				near_node = *it;
			}
			it++;
		}
		return near_node;
	}
	else
		return NULL;
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

bool SMP::checkCollision(Nodes n1, Nodes n2, const list<obstacles> obst)
{
	ofVec2f temp = n2.location - n1.location;
	float m = temp.y / temp.x;
	float c = n1.location.y - m* n1.location.x;
	for (auto i : obst) {
		float dist = abs(m*i.loc().x - i.loc().y + c) / hypot(m, 1);
		if (dist < i.rad()) {
			return false;
		}
	}
	return true;
}



bool SMP::checkSample(Nodes n, const list<obstacles> obst)
{
	for (auto i : obst) {
		if (n.location.distance(i.loc()) <= i.rad()) return false;
	}
	return true;
}

std::list<Nodes*> RRTstar::findClosestNeighbours(Nodes u, float radius, std::list<Nodes>& nodes)
{
	std::list<Nodes*> closestNeighbours;
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		if (u.location.squareDistance((*it).location) < radius)
			closestNeighbours.push_back(&(*it));
		it++;
	}
	return closestNeighbours;
}