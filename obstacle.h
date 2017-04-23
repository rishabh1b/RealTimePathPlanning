#pragma once
#include"simulationParam.h"
class obstacles
{
public:
	obstacles();
	~obstacles();
	void move();
private:
	ofVec2f location;
	float radius;
	ofColor color;
};