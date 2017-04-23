#pragma once
#include"simulationParam.h"
#include"nodeStruct.h"
#include<list>
//--------------------------------------------------------------class
class Enviroment
{
public:
	//--------------------------------------------------------------Function
	// Default constructor  
	Enviroment() {};
	// Default destructor  
	~Enviroment() {};
	// Setup method
	void setup();
	// Update method
	void update();
	// Render method draw nodes in enviroment.
	void render();

	void renderGrid();
	//--------------------------------------------------------------Variables
	bool grid = false;
private:
	//--------------------------------------------------------------Variables
protected:
	//--------------------------------------------------------------Variables
	std::list<Nodes> nodes;
	// A list or an array of Obstacles should come here

};

inline void Enviroment::setup()
{
	
}

inline void Enviroment::update()
{
}

inline void Enviroment::render()
{
	if (grid == true) renderGrid();
	ofEnableAlphaBlending();
	for (size_t i = 0; i < TotalNodes; i++)
	{
		float x = ofRandom(0, ofGetWindowWidth());
		float y = ofRandom(0, ofGetWindowHeight());

		Nodes node; 

		node.location.set(x,y);
		int hue = node.alive ? 130 : 80;
		ofSetColor(node.color, hue);

		ofDrawCircle(node.location.x, node.location.y, NODE_RADIUS);
	}

	ofDisableAlphaBlending();
}

inline void Enviroment::renderGrid()
{
	ofEnableAlphaBlending();
	ofSetColor(20, 130, 0, 50);
	for (int i = 0; i < ofGetWindowWidth(); i += 5)
	{
		ofDrawLine(i, 0, i, ofGetWindowHeight());
	}
	for (int j = 0; j < ofGetWindowHeight(); j += 5)
	{
		ofDrawLine(0, j, ofGetWindowWidth(), j);
	}
	ofDisableAlphaBlending();
}