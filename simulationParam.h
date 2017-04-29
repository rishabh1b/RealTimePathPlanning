#pragma once
#include "ofMain.h"

#include <limits>
//#define randomSeed 5
//#define CLK
//--------------------------------------------------------------Macros
#define M(e) ofMap(e,-1,1,0,1);
const float inf = std::numeric_limits<float>::infinity();
//--------------------------------------------------------------ofApp.h
#define numberOfobst 19
//--------------------------------------------------------------obstracle.h
//#define manual
#define obstMaxVelocity 1
#ifndef manual
#define automatic
#endif // !manual
//--------------------------------------------------------------Enviroment.h
#define NODE_RADIUS 3
//--------------------------------------------------------------Robot.h
#define sensorRadius 100
#define mVal 4
#define mForce 1
#define accur 1.0
//-------------------------------------------------------------Gen
#define startx 500
#define starty 500
#define goalx 250
#define goaly 200
#define converge 50.0
#define epsilon 20
#define rrtstarradius 25
#define allowedTimeRewiring 0.033
#define maxNeighbours 10
#define minDistClosestNode 5
#define alpha 0.1
#define beta 1.4