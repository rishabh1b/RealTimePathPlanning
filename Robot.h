#pragma once

#pragma once
#include"simulationParam.h"
#include"nodeStruct.h"
#include"obstacle.h"
class Robot
{
public:
	//--------------------------------------------------------------Function
	// Default constructor  
	Robot() { setup(); }
	Robot(ofVec2f loc) { setup(loc);}
	// Default destructor  
	~Robot() {};
	void setup();
	void setup(ofVec2f loc);
	// Update method
	void update();
	//Render method
	void render();
	//Compute force addition
	void addForce(ofVec2f force);
	//Controller genrate force toward target
	void controller(ofVec2f target);
	//Find Path from assign node
	//void fly(Nodes *&nodes);
	// Return state of Robot
	bool isAlive() { return alive; }
	// Return X cordinate
	float x() { return location.x; }
	// Return Y cordinate
	float y() { return location.y; }
	// Return scanning accuracy of Robot
	float accu() { return accuracy; }
	// Return scanning radius of Robot
	float getScanRadius() { return scanRadius; }
	// Return Location of Robot
	ofVec2f getLocation() { return location; }
	//Return Color of Robot
	ofColor getColor() { return color; }
	void fillEnviroment(const list<obstacles*> obst,list<Nodes> &node);
	void updateEnviroment(list<Nodes> &node, obstacles *obst);
	//--------------------------------------------------------------Variables
private:
	bool alive;
	float scanRadius, mass, accuracy;

	ofColor color;
	ofVec2f HOME, location, velocity, accelaration, maxVelocity, maxForce;
	ofPolyline line;
	ofPoint pt;
};

