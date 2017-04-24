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
	std::list<Nodes*> safeNeighbours;
protected:
	//--------------------------------------------------------------Variables
	std::list<Nodes> nodes;
	std::list<obstacles> obst;
	RRTstar rrtstar;
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

}

inline void Enviroment::update()
{

	std::list<obstacles>::iterator itobs = obst.begin();
	while (itobs != obst.end()) {
		itobs->move();
		itobs++;
	}

	Nodes u;
	do
	{
		u = SMP::sampler();

	} while (!SMP::checkSample(u,obst));

	Nodes* v = SMP::nearestNode(u, (this->nodes));
	double dist = u.location.distance((*v).location);
	if (dist > epsilon)
	{
		u.location+=(u.location - v->location).normalized() * epsilon;
		//float x_n = v->location.x + (u.location.x - v->location.x)  * epsilon / dist;
		//float y_n = v->location.y + (u.location.y - v->location.y)  * epsilon / dist;
		//u.location.x = x_n;
		//u.location.y = y_n;
	}
	std::list<Nodes*> closestNeighbours = rrtstar.findClosestNeighbours(u, rrtstarradius, (this->nodes));

	if (closestNeighbours.empty())
		return;

	std::list<Nodes*>::iterator it = closestNeighbours.begin();
	while (it != closestNeighbours.end()) // What if there is only one element?
	{
		if (SMP::checkCollision(u, *(*it),obst))
			this->safeNeighbours.push_back(*it);
		it++;
	}
	if (safeNeighbours.empty())
		return;

	Nodes* parent = SMP::nearestNode(u, safeNeighbours);
	if (parent != NULL)
	{
		u.parent = parent;
		u.costToStart = parent->costToStart + parent->location.distance(u.location);
	}
	SMP::addNode(u, (this->nodes));
	it = safeNeighbours.begin();
	while (it != safeNeighbours.end())
	{
		if ((*it)->costToStart > u.costToStart + u.location.distance((*it)->location))
		{
			(*it)->parent = &u;
			(*it)->costToStart = u.costToStart + u.location.distance((*it)->location);
		}
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
			std::cout << i.parent->location.x<<"       "<<i.parent->location.y << endl;
			line.draw();
		}
		int hue = i.alive ? 130 : 80;
		ofSetColor(i.color, hue);
		//ofDrawCircle(i.location.x, i.location.y, NODE_RADIUS);
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