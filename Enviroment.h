#pragma once
#include"simulationParam.h"
#include"nodeStruct.h"
#include"obstacle.h"
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
	std::list<obstacles> obst;
	// A list or an array of Obstacles should come here

};

inline void Enviroment::setup()
{
	for (unsigned int i = 0; i < numberOfobst; i++)
	{
		obstacles ob;
		obst.push_back(ob);
	}
}

inline void Enviroment::update()
{
	std::list<obstacles>::iterator it = obst.begin();
	while (it != obst.end()) {
		it->move();
		it++;
	}
}

inline void Enviroment::render()
{
	ofEnableAlphaBlending();

	for (auto i : obst) {
		i.render();
		//cout << i.getX() << "  " << i.getY() << endl;
	}
	for (auto i : this->nodes)
	{
		ofSetColor({ 10,10,10 });

		if (i.parent != NULL) {
			ofPoint pt;ofPolyline line;
			
			pt.set(i.location.x, i.location.y);line.addVertex(pt);
			pt.set(i.parent->location.x, i.parent->location.y);line.addVertex(pt);
			
			line.draw();
		}
		int hue = i.alive ? 130 : 80;
		ofSetColor(i.color, hue);
		ofDrawCircle(i.location.x, i.location.y, NODE_RADIUS);
	}
	ofDisableAlphaBlending();
}

//inline void Enviroment::render()
//{
//	if (grid == true) renderGrid();
//	ofEnableAlphaBlending();
//	for (size_t i = 0; i < TotalNodes; i++)
//	{
//		float x = ofRandom(0, ofGetWindowWidth());
//		float y = ofRandom(0, ofGetWindowHeight());
//
//		Nodes node; 
//
//		node.location.set(x,y);
//		int hue = node.alive ? 130 : 80;
//		ofSetColor(node.color, hue);
//
//		ofDrawCircle(node.location.x, node.location.y, NODE_RADIUS);
//	}
//
//	ofDisableAlphaBlending();
//}

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