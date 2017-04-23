#pragma once
#include "nodeStruct.h"
#include "ofMain.h"
#include "Enviroment.h"

class SMP : public Enviroment
{
public:
	SMP();
	void addNode(Nodes n);
	Nodes* nearestNode(Nodes n);
	bool checkCollision(Nodes n1, Nodes n2);
	bool checkSample(Nodes n);
	Nodes sampler();
};