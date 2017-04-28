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
	//float stepsize = ofRandom(0, 10);
	//float stepx = ofRandom(-stepsize, stepsize);
	//float stepy = ofRandom(-stepsize, stepsize);
	//location.x += stepx;
	//location.y += stepy;
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

movingObst::movingObst()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	location.set(x, y);
	maxVal = obstMaxVelocity;
#ifdef automatic
	velocity.set(maxVal, maxVal);
#endif // automatic
	radius = 30;
	color = { 200,100,20 };
}

movingObst::~movingObst()
{
}

void movingObst::render()
{
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	ofDrawCircle(location.x, location.y, radius);
	ofNoFill();
	ofDisableAlphaBlending();
}
#ifdef manual
void movingObst::move(char key)
{
	if (key == 'w')
	{
		location.y -= maxVal;
	}
	else if (key == 's')
	{
		location.y += maxVal;
	}
	if (key == 'a')
	{
		location.x -= maxVal;
	}
	else if (key == 'd')
	{
		location.x += maxVal;
	}
}
#endif // 
#ifdef automatic
void movingObst::move()
{
	if (location.y+radius >= ofGetHeight() || location.y- radius <= 0) {
		velocity.y = velocity.y*-1;
	}
	if (location.x- radius <= 0 || location.x+ radius >= ofGetWidth()) {
		velocity.x = velocity.x*-1;
	}

	location += velocity;
}
#endif // automatic
