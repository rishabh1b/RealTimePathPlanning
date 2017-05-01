#include "obstacle.h"

obstacles::obstacles()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	location.set(x, y);
	radius = ofRandom(10, 20);
	color = { 200,50,10 };
}

obstacles::obstacles(ofVec2f loc)
{
	location = loc;
	radius = ofRandom(10, 20);
	color = { 200,50,10 };
}

obstacles::~obstacles()
{
}

void obstacles::move(std::list <obstacles *> obst)
{
	//float stepsize = ofRandom(0, 10);
	//float stepx = ofRandom(-stepsize, stepsize);
	//float stepy = ofRandom(-stepsize, stepsize);
	//location.x += stepx;
	//location.y += stepy;
}


void obstacles::render()
{
	//move();
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	ofDrawCircle(location.x, location.y, radius);
	ofNoFill();
	ofDisableAlphaBlending();
}

bool obstacles::isCollide(ofVec2f n1, ofVec2f n2)
{
	float x1 = n1.x;
	float x2 = n2.x;
	float y1 = n1.y;
	float y2 = n2.y;

	float xo = location.x;
	float yo = location.y;
	float lambda = std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2);
	float t = (std::pow(x1, 2) + x2 * xo - x1*(x2 + xo) - (yo - y1)*(y1 - y2)) / lambda;
	float shortest_dist;
	if (t >= 0 && t <= 1) // If the perpendicular distance lies on the line 'segment' connecting point_1 and point_2
		shortest_dist = std::sqrt(std::pow((x2 * (y1 - yo) + x1 * (yo - y2) + xo * (y2 - y1)), 2) / lambda);
	else  // If not then only check for the end-points of the segment for the collision
	{
		float d1 = std::sqrt(std::pow((x1 - xo), 2) + std::pow((y1 - yo), 2));
		float d2 = std::sqrt(std::pow((x2 - xo), 2) + std::pow((y2 - yo), 2));
		shortest_dist = std::min(d1, d2);
	}

	if (shortest_dist < radius) 	return true;
	return false;
}

bool obstacles::isInside(ofVec2f n)
{
	return (n.distance(location) <= radius);
}

movingObst::movingObst()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	location.set(x, y);
	maxVal = obstMaxVelocity;
#ifdef automatic
	velocity.set(maxVal, maxVal);
#endif // automatic
	radius = 30;
	color = { 200,100,20 };
}

movingObst::~movingObst()
{
}

void movingObst::render()
{
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	ofDrawCircle(location.x, location.y, radius);
	ofNoFill();
	ofDisableAlphaBlending();
}
#ifdef manual
void movingObst::move(char key)
{
	if (key == 'w')
	{
		location.y -= maxVal;
	}
	else if (key == 's')
	{
		location.y += maxVal;
	}
	if (key == 'a')
	{
		location.x -= maxVal;
	}
	else if (key == 'd')
	{
		location.x += maxVal;
	}
}
#endif // 
#ifdef automatic
void movingObst::move(std::list <obstacles *> obst)
{
	ofVec2f acceleration;
	acceleration.set(0,0);
	for (auto i : obst) {
		acceleration += repulse(*i);
	}
	velocity = velocity + acceleration;
	if (velocity.length() > maxVal) {
		velocity.x = maxVal;
		velocity.y = maxVal;
	}
	if (location.y + radius >= ofGetHeight() || location.y - radius <= 0) {
		velocity.y = velocity.y*-1;
	}
	if (location.x - radius <= 0 || location.x + radius >= ofGetWidth()) {
		velocity.x = velocity.x*-1;
	}
	location += velocity;

}
#endif // automatic
bool movingObst::isCollide(ofVec2f n1, ofVec2f n2)
{
	float x1 = n1.x;
	float x2 = n2.x;
	float y1 = n1.y;
	float y2 = n2.y;

	float xo = location.x;
	float yo = location.y;
	float lambda = std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2);
	float t = (std::pow(x1, 2) + x2 * xo - x1*(x2 + xo) - (yo - y1)*(y1 - y2)) / lambda;
	float shortest_dist;
	if (t >= 0 && t <= 1) // If the perpendicular distance lies on the line 'segment' connecting point_1 and point_2
		shortest_dist = std::sqrt(std::pow((x2 * (y1 - yo) + x1 * (yo - y2) + xo * (y2 - y1)), 2) / lambda);
	else  // If not then only check for the end-points of the segment for the collision
	{
		float d1 = std::sqrt(std::pow((x1 - xo), 2) + std::pow((y1 - yo), 2));
		float d2 = std::sqrt(std::pow((x2 - xo), 2) + std::pow((y2 - yo), 2));
		shortest_dist = std::min(d1, d2);
	}

	if (shortest_dist < radius) 	return true;
	return false;
}
bool movingObst::isInside(ofVec2f n)
{
	return (n.distance(location) <= radius);
}
ofVec2f movingObst::repulse(obstacles obst)
{
	ofVec2f acceleration = location - obst.loc();
	float dist = acceleration.length();
	float strength = repulseConst / (dist*dist);
	acceleration = acceleration*strength;
	return acceleration;
}
maze::maze(ofVec2f loc)
{
	location = loc;
	color = { 10,10,50 };
	rect.height = 0.40*ofGetHeight();
	rect.width = 20;
	rect.x = loc.x;
	rect.y = loc.y;
	
}

maze::~maze()
{
}

void maze::render()
{
	ofEnableAlphaBlending();
	ofSetColor(color);
	ofFill();
	//ofRect(location.x, location.y, width, height);
	ofDrawRectangle(rect);
	ofNoFill();
	ofDisableAlphaBlending();
}

void maze::move(std::list <obstacles *> obst)
{

}

bool maze::isCollide(ofVec2f p1, ofVec2f p2)
{
	return rect.intersects(p1,p2);
}

bool maze::isInside(ofVec2f p)
{
	return rect.inside(p);
}
