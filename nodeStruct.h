#pragma once
#include"ofMain.h"
struct Nodes
{
	ofVec2f location;
	
	Nodes *parent, *prevParent=NULL;

	bool alive = true;
	ofColor color = { 10, 12, 160 };
	float costToStart;
	std::list<Nodes*> children;
	Nodes()
	{

	}
	Nodes(float x_, float y_, float costToStart_, Nodes* p_ = NULL)
	{
		location.x = x_;
		location.y = y_;
		costToStart = costToStart_;
		parent = p_;
	}

};