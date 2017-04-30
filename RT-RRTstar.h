#pragma once
#include<queue>
#include"Robot.h"

struct nodes_compare {
	bool operator() (const Nodes* lhs, const Nodes* rhs) const {
		if (lhs->location.x != rhs->location.x)
			return lhs->location.x < rhs->location.x;
		else
			return lhs->location.y < rhs->location.y;
	}
};

class RTRRTstar : public InformedRRTstar
{
public:
	void nextIter(std::list<Nodes> &nodes, const std::list<obstacles*>& obst, Robot* agent);
	static std::set<Nodes*, nodes_compare> visited_set;
	static bool goalDefined;
	std::list<Nodes*> currPath;
private:
	void expandAndRewire(std::list<Nodes>& nodes, const std::list<obstacles*>& obst);
	void updateNextBestPath();
	Nodes sample();
	Nodes* getClosestNeighbour(Nodes u, std::list<Nodes>& nodes);
	void addNode(Nodes n, Nodes* closest, std::list<Nodes>& nodes, const std::list<obstacles*>& obst);
	void rewireRandomNode(const list<obstacles*> &obst, std::list<Nodes> &nodes);
	void rewireFromRoot(const list<obstacles*> &obst, std::list<Nodes> &nodes);
	float cost(Nodes* node);
	float getHeuristic(Nodes* u);
	void changeRoot(Nodes* nextPoint, std::list<Nodes>& nodes);
	bool isPathToGoalAvailable();
	//std::list<Nodes*> RTRRTstar::findNodesNear(Nodes u);

	std::list<Nodes*> rewireRand;
	std::list<Nodes*> rewireRoot;
	std::list<Nodes*> closestNeighbours;

	std::list<Nodes*> pushedToRewireRoot;

	float timeKeeper;
	// Add a grid based indexing DS for nodes
};

