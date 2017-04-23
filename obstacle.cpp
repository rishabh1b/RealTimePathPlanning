#include "obstacle.h"

obstacles::obstacles()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	location.set(x, y);
	radius = ofRandom(10, 20);
	color = { 200,50,10 };
}

obstacles::obstacles(ofVec2f loc)
{
	location = loc;
	radius = ofRandom(10, 20);
	color = { 200,50,10 };
}

obstacles::~obstacles()
{
}

void obstacles::move()
{
	float stepsize = ofRandom(0, 10);
	float stepx = ofRandom(-stepsize, stepsize);
	float stepy = ofRandom(-stepsize, stepsize);
	location.x += stepx;
	location.y += stepy;
}


void obstacles::render()
{
	//move();
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	ofDrawCircle(location.x, location.y, radius);
	ofNoFill();
	ofDisableAlphaBlending();
}
