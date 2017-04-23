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
	if (!nodes.empty()) {
		Nodes* nearNode = & nodes.front();
		float minDist = n.location.distanceSquared(nearNode->location);
		for (auto index : nodes) {
			float dist = n.location.squareDistance(index.location);
			if (dist < minDist) {
				minDist = dist;
				nearNode = &index;
			}
		}
		return nearNode;
	}
	else
	{
		return NULL;
	}
}

bool SMP::checkCollision(Nodes n1, Nodes n2, const list<obstacles> obst)
{
	ofVec2f temp= n2.location - n1.location;
	float m = temp.y / temp.x;
	float c = n1.location.y - m* n1.location.x;
	for (auto i : obst) {
		float dist = abs(m*i.loc().x - i.loc().y + c) / hypot(m, 1);
		if (dist < i.rad()) {
			return false;
		}
	}
	return false;
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

bool SMP::checkSample(Nodes n, const list<obstacles> obst)
{
	for (auto i : obst) {
		if (n.location.distance(i.loc()) <= i.rad()) return false;
	}
	return true;
}