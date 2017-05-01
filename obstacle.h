#pragma once
#include"simulationParam.h"
class obstacles
{
public:
	obstacles();
	obstacles(ofVec2f loc);
	~obstacles();
#ifdef automatic
	virtual void move(std::list<obstacles*> obst);
	virtual void applyForce(ofVec2f force);
	virtual void update();
	virtual ofVec2f repulsive(obstacles *obst);
#endif // automatic
	virtual void render();
	virtual ofVec2f loc(){ return location; }
	virtual float rad() { return radius; }
	float getX() { return location.x;}
	float getY() { return location.y;}
	virtual  bool isCircle() { return true; }
	virtual bool isCollide(ofVec2f, ofVec2f);
	virtual bool isInside(ofVec2f);
	float mass;
private:
	ofVec2f location,velocity,accelaration;
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
	void move(std::list<obstacles*> obst);
#endif // automatic
	ofVec2f loc() { return this->location; }
	float rad() { return this->radius; }
	bool isCircle() { return true; }
	bool isCollide(ofVec2f, ofVec2f);
	bool isInside(ofVec2f);
	void applyForce(ofVec2f force);
	void update();
	ofVec2f repulsive(obstacles *obst);
private:
	ofVec2f location, accelaration;
	float radius;
	ofColor color;
	float maxVal;
#ifdef automatic
	ofVec2f velocity;
#endif // automatic
};

class maze:public obstacles
{
public:
	maze(ofVec2f loc);
	maze(ofVec2f loc, float width, float height);
	~maze();
	void render();
#ifdef automatic
	void move(std::list<obstacles*> obst);
#endif 
	ofVec2f loc();
	bool isCircle() { return false; }
	bool isCollide(ofVec2f p1, ofVec2f p2);
	bool isInside(ofVec2f p);
private:
	ofColor color;
	ofVec2f location;
	float width, height;
	ofRectangle rect;
};