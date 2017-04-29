#pragma once
#include "nodeStruct.h"
#include "ofMain.h"
#include "obstacle.h"
#include<set>

class SMP
{
public:
	SMP();
	static void addNode(Nodes n, std::list<Nodes>& nodes);
	static Nodes* nearestNode(Nodes n, std::list<Nodes>& nodes);
	static Nodes* nearestNode(Nodes n, std::list<Nodes*>& nodes);
	static bool checkCollision(Nodes n1, Nodes n2, list<obstacles*> obst);
	static bool checkSample(Nodes n, list<obstacles*> obst);
	static Nodes sampler();
	static bool goalFound;
	static bool sampledInGoalRegion;
	static bool moveNow;
	static ofVec2f SMP::start;
	static ofVec2f goal;
	static Nodes* SMP::root;
	static Nodes* SMP::target;
	static Nodes* SMP::nextTarget;
};