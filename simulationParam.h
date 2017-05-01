#pragma once
#include "ofMain.h"

#include <limits>
//#define randomSeed 5
//#define CLK
//--------------------------------------------------------------Macros
#define M(e) ofMap(e,-1,1,0,1);
const float inf = std::numeric_limits<float>::infinity();
//--------------------------------------------------------------ofApp.h
#define numberOfobst 20
//--------------------------------------------------------------obstracle.h
//#define manual
#define obstMaxVelocity 0.2
#ifndef manual
#define automatic
#endif // !manual
//--------------------------------------------------------------Enviroment.h
#define NODE_RADIUS 3
//--------------------------------------------------------------Robot.h
#define sensorRadius 200
#define mVal 4
#define mForce 1
#define accur 1.0
//-------------------------------------------------------------Genral
#define startx 500
#define starty 500
#define goalx 250
#define goaly 200
#define converge 50.0
#define epsilon 25
#define rrtstarradius 50
#define allowedTimeRewiring 0.5
#define maxNeighbours 50
#define minDistClosestNode 5
#define alpha 0.1
#define beta 1.4