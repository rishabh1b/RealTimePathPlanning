#pragma once
#include "ofMain.h"
//#define randomSeed 5
//#define CLK
//--------------------------------------------------------------Macros
#define M(e) ofMap(e,-1,1,0,1);
//--------------------------------------------------------------ofApp.h
#define numberOfobst 20
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
#define epsilon 5
#define rrtstarradius 35