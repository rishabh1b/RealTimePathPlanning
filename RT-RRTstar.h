#pragma once
class RTRRTstar : public InformedRRTstar
{
void nextIter(std::list<Nodes> &nodes, const std::list<obstacles>& obst, Nodes* u_ = NULL);
private:
	void expandAndRewire(std::list<Nodes>& nodes, const std::list<obstacles>& obst);
	void updateNextBestPath();
	Nodes sample();
	Nodes* getClosestNeighbour(Nodes u);
	void RTRRTstar::addNode(Nodes n, Nodes* closest, std::list<Nodes>& nodes);
	float cost(Nodes* node);
	//std::list<Nodes*> RTRRTstar::findNodesNear(Nodes u);

	std::list<Nodes*> currPath;
	std::list<Nodes*> rewireRand;
	std::list<Nodes*> rewireRoot;
	std::list<Nodes*> closestNeighbours;

	bool pathAvailable;
	// Add a grid based indexing DS for nodes
};

