#pragma once
#include"simulationParam.h"
#include"nodeStruct.h"
#include"obstacle.h"
#include<list>
#include"SMP.h"
#include"RRTstar.h"
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
	std::list<Nodes> path;
	RRTstar rrtstar;
	bool flag = false;
	ofVec2f goal;
	Nodes* target=NULL;
	// A list or an array of Obstacles should come here

};

inline void Enviroment::setup()
{

	for (unsigned int i = 0; i < numberOfobst; i++)
	{
		obstacles ob;
		obst.push_back(ob);
	}

	Nodes start(startx, starty, 0);
	this->nodes.push_back(start);
	goal.set(goalx, goaly);
}

inline void Enviroment::update()
{

	/*std::list<obstacles>::iterator itobs = obst.begin();
	while (itobs != obst.end()) {
		itobs->move();
		itobs++;
	}*/


	rrtstar.nextIter(nodes, obst);
	std::list<Nodes>::iterator it = nodes.begin();
	while (it != nodes.end())
	{
		if (it->location.distance(goal) < converge)
		{
			flag = !flag;
			target = &(*it);
			//std::cout << "Found" << endl;
		}
		it++;
	}
	if (flag)
	{
		path.clear();
		Nodes *pathNode = target;
		do
		{
			path.push_back(*pathNode);
			pathNode = pathNode->parent;
		} while (pathNode->parent != NULL);
	}
	//rrtstar.nextIter(nodes, obst);
}

inline void Enviroment::render()
{
	ofEnableAlphaBlending();
	for (auto i : obst) {
		i.render();
		//cout << i.getX() << "  " << i.getY() << endl;
	}

	ofSetColor({200, 200, 10});
	ofFill();
	ofDrawCircle(goal.x, goal.y, NODE_RADIUS);
	ofNoFill();
	ofDrawCircle(goal.x, goal.y, converge);

	for (auto i : this->nodes)
	{
		ofSetColor({ 10,10,10 });

		if (i.parent != NULL) {
			ofPoint pt;ofPolyline line;
			pt.set(i.location.x, i.location.y);line.addVertex(pt);
			pt.set(i.parent->location.x, i.parent->location.y);line.addVertex(pt);
			line.draw();
		}
		ofSetColor({ 10,250,250 });
		if (i.prevParent != NULL) {
			
			ofPoint pt; ofPolyline line;
			pt.set(i.location.x, i.location.y); line.addVertex(pt);
			pt.set(i.prevParent->location.x, i.prevParent->location.y); line.addVertex(pt);
			line.draw();
		}
		int hue = i.alive ? 130 : 80;
		ofSetColor(i.color, hue);
		//ofDrawCircle(i.location.x, i.location.y, NODE_RADIUS);
	}
	if (!path.empty())
	{
		ofSetColor({ 20,250,20 });
		ofSetLineWidth(5);
		for (auto i : path) {
			if (i.parent != NULL) {
				ofPoint pt; ofPolyline line;
				pt.set(i.location.x, i.location.y); line.addVertex(pt);
				pt.set(i.parent->location.x, i.parent->location.y); line.addVertex(pt);
				line.draw();
			}
		}
		ofSetLineWidth(1);
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