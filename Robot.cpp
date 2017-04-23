#include "Robot.h"

void Robot::setup()
{
		alive = true; mass = 5.0; scanRadius = sensorRadius; accuracy = accur;
		//battery = 100;
		//float x = ofRandom(0, ofGetWindowWidth()); float y = ofRandom(0, ofGetWindowHeight());`
		location.set(0.0,0.0); HOME = location;
		velocity.set(0.0, 0.0);
		accelaration.set(0.0, 0.0);
		maxVelocity.set(mVal, mVal);
		maxForce.set(mForce, mForce);
		color = { ofRandom(0,255),ofRandom(0,255) ,ofRandom(0,255) };
}

void Robot::update()
{
	velocity += accelaration;
	velocity = (velocity.length() <= maxVelocity.length()) ? velocity : (velocity.normalized() *mVal);
	location += velocity;
	accelaration *= 0.0;
	pt.set(location.x, location.y);
	line.addVertex(pt);
}

void Robot::render()
{

	int r = 6;
	ofEnableAlphaBlending();
	ofFill();
	ofSetColor(color);
	this->line.draw();
	//ofDrawCircle(location.x(), location.y(), r);
	ofNoFill();
	ofSetColor(color);
	ofDrawCircle(location.x,location.y,scanRadius);
	ofPushMatrix();
	ofTranslate(location.x,location.y);
	//char Str[3]; // an array of chars
	////ofSetColor({ 0,0,0 });
	//sprintf(Str, "%d", battery);
	//myfont.drawString(Str, -10, scanRadius+2);
	ofRotate(ofRadToDeg(atan2(velocity.y, velocity.x) + PI / 2));
	ofFill();
	ofBeginShape();
	ofVertex(0, -r * 2);
	ofVertex(-r, r * 2);
	ofVertex(r, r * 2);
	ofEndShape(true);
	ofPopMatrix();
	//ofNoFill();
	//ofDrawCircle(location.x, location.y, scanRadius-3);
	ofNoFill();
	//for (float i = 0; i <= 0.1; i += 0.05) {
	//	ofDrawCircle(location.x(), location.y(), sin(i*ofGetFrameNum())*scanRadius);
	//	//ofDrawCircle(0, 0, sin(i*ofGetFrameNum())*scanRadius);
	//}
	//ofDrawBitmapString("quadA", ofGetMouseX(), ofGetMouseY() + 52);
	ofDisableAlphaBlending();
}

void Robot::addForce(ofVec2f force)
{
	accelaration += (force / mass);
}

void Robot::controller(ofVec2f target)
{
	ofVec2f error = (target - location);
	//error.normalize();
	//error *= 1.5;
	//accelaration = error;
	ofVec2f temp = error.normalized()*mVal;
	ofVec2f steer = (temp - velocity);
	steer = (steer.length() <= maxForce.length()) ? steer : (steer.normalized() *mForce);
	addForce(steer);
}
//
//inline void quadCopter::fly(Nodes *& nodes)
//{
//	if (!wTraj.empty())
//	{
//		int index = 0;
//		bool prio = false;
//		Vector2f temp('inf', 'inf');
//		for (auto i : wTraj)
//		{
//			if (nodes->waypoint[i.index].alive)
//			{
//				if (!nodes->waypoint[i.index].stress) nodes->waypoint[i.index].color = color;
//				else nodes->waypoint[i.index].color = { 255,0,0 };
//				Vector2f min = this->location - nodes->waypoint[i.index].location;
//				if (min.norm()<temp.norm() && nodes->waypoint[i.index].priority)
//				{
//					temp = min;
//					index = i.index;
//					prio = true;
//				}
//			}
//		}
//		if (prio) {
//			controller(nodes->waypoint[index].location);
//		}
//		//else if (!prio) {
//		//	int index = 0;
//		//	Vector2f temp('inf', 'inf');
//		//	for (auto i : wTraj)
//		//	{
//		//		if (nodes->waypoint[i.index].alive)
//		//		{
//		//			if (!nodes->waypoint[i.index].stress) nodes->waypoint[i.index].color = color;
//		//			else nodes->waypoint[i.index].color = { 255,0,0 };
//		//			Vector2f min = this->location - nodes->waypoint[i.index].location;
//		//			if (min.norm()<temp.norm() && nodes->waypoint[i.index].priority)
//		//			{
//		//				temp = min;
//		//				index = i.index;
//		//				prio = true;
//		//			}
//		//		}
//		//	}
//		//	controller(nodes->waypoint[index].location);
//		//}
//
//		//list<waypoints>::iterator it;
//		//waypoints target = findAlive(wTraj,nodes);
//		//controller(target.location);
//
//	}
//	else
//	{
//		//Vector2f mouse(ofGetMouseX(), ofGetMouseY());
//		//controller(mouse);
//		Vector2f HOME(0, 0);
//		controller(HOME);
//	}
//}

//
//inline waypoints quadCopter::findAlive(list<waypoints> &TRAJ, node *nodes)
//{
//	list<waypoints>::iterator it;
//	it = TRAJ.begin();
//	if (TRAJ.size() <= 1)
//	{
//		return *it;
//	}
//	if (nodes->waypoint[it->index].alive)
//	{
//		return *it;
//	}
//	else
//	{
//		TRAJ.pop_front();
//		return findAlive(TRAJ, nodes);
//	}
//}