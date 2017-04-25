#include "SMP.h"
#include "RRTstar.h"
#include "simulationParam.h"

bool SMP::goalFound = false;
ofVec2f SMP::goal;
Nodes* SMP::target = NULL;

SMP::SMP()
{

}

void SMP::addNode(Nodes n, std::list<Nodes>& nodes)
{
	nodes.push_back(n);
	if (n.location.distance(goal) < converge)
	{
		goalFound = true;
		target = &(nodes.back());
	}
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

void RRTstar::nextIter(std::list<Nodes>& nodes,const list<obstacles> obst)
{

	Nodes u = SMP::sampler();
	Nodes* v = SMP::nearestNode(u, nodes);
	double dist = u.location.distance((*v).location);

	if (dist > epsilon)
	{
		float x_n = v->location.x + (u.location.x - v->location.x)  * epsilon / dist;
		float y_n = v->location.y + (u.location.y - v->location.y)  * epsilon / dist;
		u.location.x = x_n;
		u.location.y = y_n;
	}
	if (!SMP::checkSample(u, obst)) return;

	std::list<Nodes*> closestNeighbours;
	closestNeighbours = this->findClosestNeighbours(u, rrtstarradius, nodes);
	//if (closestNeighbours.empty()) {
	//	closestNeighbours.push_back(v);
	//}
	if (closestNeighbours.empty()) return;

	std::list<Nodes*>::iterator it = closestNeighbours.begin();
	std::list<Nodes*> safeNeighbours;
	while (it != closestNeighbours.end())
	{
		if (SMP::checkCollision(u, *(*it), obst))
			safeNeighbours.push_back(*it);
		it++;
	}
	if (safeNeighbours.empty()) return;

	it = safeNeighbours.begin();

	float minDist = (*it)->costToStart + u.location.distance((*it)->location);
	auto index = it;

	while (it != safeNeighbours.end())
	{
		float dist = (*it)->costToStart + u.location.distance((*it)->location);
		if (dist < minDist) {
			minDist = dist;
			index = it;
		}
		it++;
	}
	u.parent = *index;
	u.costToStart = minDist;

	SMP::addNode(u, nodes);

	safeNeighbours.remove(*index);

	// Bring the iterator to initial position again
	it = safeNeighbours.begin();
	while (it != safeNeighbours.end())
	{
		float dist = u.costToStart + u.location.distance((*it)->location);
		if ((*it)->costToStart > dist)
		{
			(*it)->prevParent = (*it)->parent;
			(*it)->parent = &(nodes.back());
			(*it)->costToStart = dist;
		}
		it++;
	}

}

std::list<Nodes*> RRTstar::findClosestNeighbours(Nodes u, float radius, std::list<Nodes>& nodes)
{
	std::list<Nodes*> closestNeighbours;
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		if (u.location.distance(it->location) < radius)
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
	return closestNeighbours;
}