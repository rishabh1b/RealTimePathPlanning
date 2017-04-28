#pragma once
#include"simulationParam.h"
class obstacles
{
public:
	obstacles();
	obstacles(ofVec2f loc);
	~obstacles();
	virtual void move();
	virtual void render();
	virtual ofVec2f loc(){ return location; }
	virtual float rad() { return radius; }
	float getX() { return location.x;}
	float getY() { return location.y;}

private:
	ofVec2f location;
	float radius;
	ofColor color;
};
class movingObst : public obstacles
{
public:
	movingObst();
	~movingObst();
	void render();
#ifdef manual
	void move(char key);
#endif // manual
#ifdef automatic
	void move();
#endif // automatic
	ofVec2f loc() { return this->location; }
	float rad() { return this->radius; }
private:
	ofVec2f location;
	float radius;
	ofColor color;
	int maxVal;
#ifdef automatic
	ofVec2f velocity;
#endif // automatic
};