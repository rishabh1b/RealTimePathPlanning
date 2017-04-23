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