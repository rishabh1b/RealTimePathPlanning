#pragma once
#include"simulationParam.h"
class obstacles
{
public:
	obstacles();
	obstacles(ofVec2f loc);
	~obstacles();
	virtual void move(std::list <obstacles *> obst);
	virtual void render();
	virtual ofVec2f loc(){ return location; }
	virtual float rad() { return radius; }
	float getX() { return location.x;}
	float getY() { return location.y;}
	virtual  bool isCircle() { return true; }
	virtual bool isCollide(ofVec2f, ofVec2f);
	virtual bool isInside(ofVec2f);
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
	void move(std::list <obstacles *> obst);
#endif // automatic
	ofVec2f loc() { return this->location; }
	float rad() { return this->radius; }
	bool isCircle() { return true; }
	bool isCollide(ofVec2f, ofVec2f);
	bool isInside(ofVec2f);
	ofVec2f repulse(obstacles obst);
private:
	ofVec2f location;
	float radius;
	ofColor color;
	float maxVal;
#ifdef automatic
	ofVec2f velocity;
	ofVec2f acceleration;
#endif // automatic
};

class maze:public obstacles
{
public:
	maze(ofVec2f loc);
	~maze();
	void render();
	void move(std::list <obstacles *> obst);
	ofVec2f getloc() { return this->location;}
	bool isCircle() { return false; }
	bool isCollide(ofVec2f p1, ofVec2f p2);
	bool isInside(ofVec2f p);
	//float getParam(obstacles* p);
private:
	ofColor color;
	ofVec2f location;
	float width, height;
	ofRectangle rect;
};