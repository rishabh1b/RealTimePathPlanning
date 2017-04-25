#pragma once
#include"simulationParam.h"
class obstacles
{
public:
	obstacles();
	obstacles(ofVec2f loc);
	~obstacles();
	void move();
	void render();
	ofVec2f loc(){ return location; }
	float rad() { return radius; }
	float getX() { return location.x;}
	float getY() { return location.y;}

private:
	ofVec2f location;
	float radius;
	ofColor color;
};