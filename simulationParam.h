#pragma once
#include "ofMain.h"
#include <limits.h>

//float max_val = numeric_limits<float>::infinity();
//#define randomSeed 5
//#define CLK
//--------------------------------------------------------------Macros
#define M(e) ofMap(e,-1,1,0,1);
//--------------------------------------------------------------ofApp.h
#define numberOfobst 30
//--------------------------------------------------------------Enviroment.h
#define NODE_RADIUS 3
//--------------------------------------------------------------Robot.h
#define sensorRadius 35
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
#define allowedTimeRewiring 0.5
#define maxNeighbours 10
#define minDistClosestNode 5
const float inf = numeric_limits<float>::infinity();
#define alpha 0.1
#define beta 1.4