#pragma once
#include"ofMain.h"
struct Nodes
{
	ofVec2f location;
	Nodes *parent;
	bool alive = true;
	ofColor color = { 10, 12, 160 };
};