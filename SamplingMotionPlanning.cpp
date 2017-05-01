#include "SMP.h"
#include "RRTstar.h"
#include "InformedRRTstar.h"
#include "simulationParam.h"
#include "RT-RRTstar.h"

bool SMP::goalFound = false;
bool SMP::sampledInGoalRegion = false;
bool SMP::moveNow = false;
bool InformedRRTstar::usingInformedRRTstar = false;
bool RTRRTstar::goalDefined = false;

ofVec2f SMP::goal;
ofVec2f SMP::start;
Nodes* SMP::target = NULL;
Nodes* SMP::nextTarget = NULL;
Nodes* SMP::root = NULL;
std::set<Nodes*, nodes_compare> RTRRTstar::visited_set;

SMP::SMP()
{

}

void SMP::addNode(Nodes n, std::list<Nodes>& nodes)
{
	nodes.push_back(n);
	if (n.location.distance(goal) < converge)
	{
		goalFound = true;
		sampledInGoalRegion = true;
		target = &(nodes.back());
	}
}

Nodes* SMP::nearestNode(Nodes n, std::list<Nodes>& nodes)
{
	double min_dist = n.location.squareDistance(nodes.front().location);
	Nodes* near_node = &(nodes.front());
	if (!nodes.empty())
	{
		std::list<Nodes>::iterator it = nodes.begin();
		while (it != nodes.end())
		{
			if (n.location.squareDistance((*it).location) < min_dist)
			{
				min_dist = n.location.squareDistance((*it).location);
				near_node = &(*it);
			}
			it++;
		}
		return near_node;
	}
	else
		return NULL;
}

Nodes* SMP::nearestNode(Nodes n, std::list<Nodes*>& nodes)
{
	double min_dist = n.location.squareDistance(nodes.front()->location);
	Nodes* near_node = nodes.front();
	if (!nodes.empty())
	{
		std::list<Nodes*>::iterator it = nodes.begin();
		while (it != nodes.end())
		{
			if (n.location.squareDistance((*it)->location) < min_dist)
			{
				min_dist = n.location.squareDistance((*it)->location);
				near_node = *it;
			}
			it++;
		}
		return near_node;
	}
	else
		return NULL;
}


Nodes SMP::sampler()
{
	float x = ofRandom(0, ofGetWindowWidth());
	float y = ofRandom(0, ofGetWindowHeight());
	Nodes new_node;
	new_node.location.x = x;
	new_node.location.y = y;
	return new_node;
}

bool SMP::checkCollision(Nodes n1, Nodes n2, list<obstacles*> obst)
{
	for (auto i : obst) {
			if (i->isCollide(n1.location, n2.location)) 	return false;
	}
	return true;
}



bool SMP::checkSample(Nodes n,  list<obstacles*> obst)
{
	for (auto i : obst) {
			if (i->isInside(n.location)) return false;		
	}
	return true;
}


void RRTstar::nextIter(std::list<Nodes>& nodes,list<obstacles*> obst, Nodes* u_)
{
	Nodes u;
	if (u_ == NULL)
		u = SMP::sampler();
	else
		u = *u_;

	Nodes* v = SMP::nearestNode(u, nodes);
	double dist = u.location.distance((*v).location);

	if (dist > epsilon)
	{
		float x_n = v->location.x + (u.location.x - v->location.x)  * epsilon / dist;
		float y_n = v->location.y + (u.location.y - v->location.y)  * epsilon / dist;
		u.location.x = x_n;
		u.location.y = y_n;
	}
	if (!SMP::checkSample(u, obst)) return;

	std::list<Nodes*> closestNeighbours;
	closestNeighbours = RRTstar::findClosestNeighbours(u, nodes);

	if (closestNeighbours.empty()) return;

	std::list<Nodes*>::iterator it = closestNeighbours.begin();
	std::list<Nodes*> safeNeighbours;
	while (it != closestNeighbours.end())
	{
		if (SMP::checkCollision(u, *(*it), obst))
			safeNeighbours.push_back(*it);
		it++;
	}
	if (safeNeighbours.empty()) return;

	it = safeNeighbours.begin();

	float minDist = (*it)->costToStart + u.location.distance((*it)->location);
	auto index = it;

	while (it != safeNeighbours.end())
	{
		float dist = (*it)->costToStart + u.location.distance((*it)->location);
		if (dist < minDist) {
			minDist = dist;
			index = it;
		}
		it++;
	}
	u.parent = *index;
	u.costToStart = minDist;

	SMP::addNode(u, nodes);

	safeNeighbours.remove(*index);

	// Bring the iterator to initial position again
	it = safeNeighbours.begin();
	while (it != safeNeighbours.end())
	{
		float dist = u.costToStart + u.location.distance((*it)->location);
		if ((*it)->costToStart > dist)
		{
			(*it)->prevParent = (*it)->parent;
			(*it)->parent = &(nodes.back());
			(*it)->costToStart = dist;
		}
		it++;
	}

}

std::list<Nodes*> RRTstar::findClosestNeighbours(Nodes u, std::list<Nodes>& nodes)
{
	std::list<Nodes*> closestNeighbours;
	std::list<Nodes>::iterator it = nodes.begin();

	/*float rrtstarradius = std::sqrt((ofGetWindowWidth() * ofGetWindowHeight() * maxNeighbours) / (3.146 * nodes.size()));
	if (rrtstarradius < minDistClosestNode)
		rrtstarradius = minDistClosestNode;*/

	while (it != nodes.end())
	{
		float f = u.location.distance(it->location);
		if (f < rrtstarradius && f > 0.0001)
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
	return closestNeighbours;
}

void InformedRRTstar::nextIter(std::list<Nodes> &nodes, std::list<obstacles*> obst)
{
	if (sol_nodes.empty())
	{
		RRTstar::nextIter(nodes, obst);
	}
	else
	{
		float min_cost = sol_nodes.front()->costToStart;
		std::list<Nodes*>::iterator it = sol_nodes.begin();
		while (it != sol_nodes.end())
		{
			if ((*it)->costToStart < min_cost)
			{
				min_cost = (*it)->costToStart;
				SMP::target = *it;
			}
			it++;
		}

		RRTstar::nextIter(nodes, obst, &InformedRRTstar::sample(min_cost));
	}
	if (SMP::sampledInGoalRegion)
		sol_nodes.push_back(&nodes.back());
}

Nodes InformedRRTstar::sample(float c_max)
{
	float c_min = SMP::goal.distance(SMP::start);


	if (std::abs(c_max - c_min) < 100 && usingInformedRRTstar) //Putting a dummy value for now - Robot might not move for some configurations with this value
		SMP::moveNow = true; //TODO: The flag will be associated with time. Should turn on when the spcified time lapses

	ofVec2f x_centre = (SMP::start + SMP::goal) / 2;
	ofVec2f dir = SMP::goal - SMP::start;
	dir = dir.getNormalized();
	float angle = std::atan2(-dir.y, dir.x); //Frame is with y pointing downwards
	float r1 = c_max / 2;
	float r2 = std::sqrt(std::pow(c_max, 2) - std::pow(c_min, 2)) / 2;

	float x = ofRandom(-1, 1);
	float y = ofRandom(-1, 1);

	float x2 = x * r1 * std::cos(angle) + y * r2 * std::sin(angle);
	float y2 = -x * r1 * std::sin(angle) + y * r2 * std::cos(angle);

	ofVec2f rot_sample, rot_trans_sample;
	rot_sample.set(x2, y2);
	rot_trans_sample = rot_sample + x_centre;

	Nodes n;
	n.location.x = rot_trans_sample.x;
	n.location.y = rot_trans_sample.y;

	return n;
}

void RTRRTstar::nextIter(std::list<Nodes> &nodes, const std::list<obstacles*>& obst, Robot* agent)
{
	timeKeeper = ofGetElapsedTimef();
	expandAndRewire(nodes, obst);
	if (SMP::goalFound)
		updateNextBestPath();
	if (currPath.size() > 1 && agent->getLocation().distance(SMP::root->location) < 0.1)
	{
		//We have to pre-increment rather than post-increment
		Nodes* nextPoint = *((++currPath.begin())); //Change the DS for path to vector?
		changeRoot(nextPoint, nodes);

		RTRRTstar::visited_set.clear();
		pushedToRewireRoot.clear();
		rewireRoot.clear();
	}
	closestNeighbours.clear();
}

void RTRRTstar::changeRoot(Nodes* nextPoint, std::list<Nodes>& nodes)
{
	nextPoint->children.push_back(SMP::root);
	nextPoint->parent = NULL;
	nextPoint->prevParent = NULL;
	nextPoint->costToStart = 0;

	SMP::root->parent = nextPoint;
	SMP::root->costToStart = SMP::root->location.distance(nextPoint->location);
	SMP::root = nextPoint;
}

void RTRRTstar::expandAndRewire(std::list<Nodes>& nodes, const std::list<obstacles*>& obst)
{
	Nodes u = sample();
	Nodes* v = RTRRTstar::getClosestNeighbour(u, nodes);
	double dist = u.location.distance((*v).location);

	if (dist > epsilon)
	{
		float x_n = v->location.x + (u.location.x - v->location.x)  * epsilon / dist;
		float y_n = v->location.y + (u.location.y - v->location.y)  * epsilon / dist;
		u.location.x = x_n;
		u.location.y = y_n;
	}

	if (!SMP::checkSample(u, obst)) return;
	if (SMP::checkCollision(u, *v, obst))
	{
		if (this->closestNeighbours.size() < maxNeighbours)//u.location.distance(v->location) > minDistClosestNode)
		{
			this->addNode(u, v, nodes, obst);
		}
		else
		{
			this->rewireRand.push_front(v);
		}
		rewireRandomNode(obst, nodes);
	}
	rewireFromRoot(obst, nodes);
}

void RTRRTstar::updateNextBestPath()
{
	std::list<Nodes*> updatedPath;
	Nodes *pathNode = target;
	if (SMP::goalFound) {
		do
		{
			currPath.push_back(pathNode);
			pathNode = pathNode->parent;
		} while (pathNode != NULL);
		currPath.reverse();
		return;
	}
	else {
		if (!goalDefined)
			return;
		Nodes* curr_node = SMP::root;
		while (!curr_node->children.empty())
		{
			std::list<Nodes*>::iterator it = curr_node->children.begin();
			Nodes* tempNode = curr_node->children.front();
			float cost_ = cost(tempNode);
			float minCost = cost_ + getHeuristic(*it);
			while (it != curr_node->children.end()) {
				cost_ = cost(*it);
				float cost_new = cost_ + getHeuristic(*it);
				if (cost_new < minCost) {
					minCost = cost_new;
					tempNode = *it;
				}
				it++;
			}
			updatedPath.push_back(tempNode);
			if (tempNode->children.empty() || cost(tempNode) == inf)
			{
				visited_set.insert(tempNode);
				break;
			}
			curr_node = tempNode;
		}
		if (currPath.size() == 0)
			currPath.push_back(SMP::root);

		if (updatedPath.back()->location.distance(SMP::goal) < currPath.back()->location.distance(SMP::goal))
			currPath = updatedPath;
	}
	
}

Nodes RTRRTstar::sample()
{
	float rand_num = ofRandom(0, 1);

	if (rand_num > 1 - alpha && SMP::target != NULL)
	{
		float x = ofRandom(SMP::root->location.x, SMP::target->location.x);
		float y = ofRandom(SMP::root->location.y, SMP::target->location.y);
		Nodes new_node;
		new_node.location.x = x;
		new_node.location.y = y;
		return new_node;
	}
	else if (rand_num >= (1 - alpha) / beta && SMP::goalFound)
	{
		return InformedRRTstar::sample(cost(SMP::target));
	}
	else
	{
		return SMP::sampler();
	}

}

Nodes* RTRRTstar::getClosestNeighbour(Nodes u, std::list<Nodes>& nodes) //Using all the nodes for the time being
{
	double min_dist = u.location.squareDistance(nodes.front().location);
	Nodes* near_node = &(nodes.front());
	std::list<Nodes>::iterator it = nodes.begin();

	/*float rrtstarradius = std::sqrt((ofGetWindowWidth() * ofGetWindowHeight() * maxNeighbours) / (3.146 * nodes.size()));
	if (rrtstarradius < minDistClosestNode)
		rrtstarradius = minDistClosestNode;*/

	while (it != nodes.end())
	{
		float dist = u.location.squareDistance((*it).location);
		if (dist < min_dist)
		{
			min_dist = dist;
			near_node = &(*it);
		}
		if (u.location.distance(it->location) < rrtstarradius) 
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
	return near_node;
}

void RTRRTstar::addNode(Nodes n, Nodes* closest, std::list<Nodes>& nodes, const std::list<obstacles*>& obst)
{
	Nodes* parent = closest;
	float c_min = cost(closest) + n.location.distance(closest->location);
	std::list<Nodes*>::iterator it = (this->closestNeighbours).begin();
	float c_new;
	while (it != closestNeighbours.end())
	{
		c_new = cost(*it) + n.location.distance((*it)->location);
		if (c_new < c_min && SMP::checkCollision(n, *(*it), obst))
		{
			c_min = c_new;
			parent = *it;
			n.costToStart = c_min;
		}
		it++;
	}
	n.parent = parent;
	nodes.push_back(n);
	parent->children.push_back(&(nodes.back()));

	if (n.location.distance(SMP::goal) < converge)
	{
		if (SMP::target == NULL || (SMP::target != NULL && SMP::target->costToStart > n.costToStart))
		{
			SMP::target = &(nodes.back());
		}
		SMP::goalFound = true;
	}
	//TODO: Add the node to the Grid based/KD-Tree Data structure

	this->rewireRand.push_front(&(nodes.back()));
}

float RTRRTstar::cost(Nodes* node)
{
	bool badNode = false;
	float cost_ = 0;
	Nodes* curr = node;
	while (curr->parent != NULL)
	{
		if (curr->parent->costToStart == inf)
		{
			node->costToStart = inf;
			badNode = true;
			break;
		}
		cost_ += curr->location.distance(curr->parent->location);
		curr = curr->parent;
	}
	if (badNode)
		return inf;
	else
	{
		node->costToStart = cost_;
		return cost_;
	}
}

void RTRRTstar::rewireRandomNode(const list<obstacles*> &obst, std::list<Nodes> &nodes)
{
	while (!rewireRand.empty() && (ofGetElapsedTimef() - timeKeeper) < 0.5 * allowedTimeRewiring)
	{
		Nodes* Xr = rewireRand.front();
		rewireRand.pop_front();

		std::list<Nodes*> nearNodes = RRTstar::findClosestNeighbours(*Xr, nodes);
		std::list<Nodes*>::iterator it = nearNodes.begin();
		std::list<Nodes*> safeNeighbours;
		while (it != nearNodes.end())
		{
			if (SMP::checkCollision(*Xr, *(*it), obst))
				safeNeighbours.push_back(*it);
			it++;
		}
		if (safeNeighbours.empty()) continue;

		it = safeNeighbours.begin();
		float cost_ = cost(Xr);
		while (it != safeNeighbours.end())
		{

			float oldCost = cost(*it);
			float newCost = cost_ + Xr->location.distance((*it)->location);
			if (newCost < oldCost)
			{

				(*it)->prevParent = (*it)->parent;
				(*it)->parent->children.remove(*it);
				(*it)->parent = Xr;
				(*it)->costToStart = newCost;
				Xr->children.push_back(*it);
				rewireRand.push_back(*it);
			}
			it++;
		}
	}
}

void RTRRTstar::rewireFromRoot(const list<obstacles*> &obst, std::list<Nodes> &nodes) {

	if (rewireRoot.empty()) {
		rewireRoot.push_back(SMP::root);
	}

	while (!rewireRoot.empty() && (ofGetElapsedTimef() - timeKeeper) < allowedTimeRewiring) 
		{
			Nodes* Xs = rewireRoot.front();
			rewireRoot.pop_front();
			std::list<Nodes*> nearNeighbours;
			nearNeighbours = RRTstar::findClosestNeighbours(*Xs, nodes);

			std::list<Nodes*>::iterator it = nearNeighbours.begin();
			std::list<Nodes*> safeNeighbours;
			while (it != nearNeighbours.end())
			{
				if (SMP::checkCollision(*Xs, *(*it), obst))
					safeNeighbours.push_back(*it);
				it++;
			}
			if (safeNeighbours.empty()) continue;

			safeNeighbours.remove(Xs->parent);
			it = safeNeighbours.begin();
			while (it != safeNeighbours.end()) {

				float oldCost = cost(*it);
				float newCost = cost(Xs) + Xs->location.distance((*it)->location);
				if (newCost < oldCost) {

					(*it)->prevParent = (*it)->parent;
					(*it)->parent->children.remove(*it);
					(*it)->parent = Xs;
					(*it)->costToStart = newCost;
					Xs->children.push_back(*it);
				}
				//TODO: take care of restarting the queue part
				bool found = std::find(pushedToRewireRoot.begin(), pushedToRewireRoot.end(), (*it)) != pushedToRewireRoot.end();
				if (!found) {
					rewireRoot.push_back((*it));
					pushedToRewireRoot.push_back((*it));
				}
				it++;
			}
		}
	}

float RTRRTstar::getHeuristic(Nodes* u) {
	if (visited_set.find(u) != visited_set.end())
		return inf;
	else
		return u->location.distance(SMP::goal);
}

//method not used
bool RTRRTstar::isPathToGoalAvailable()
{
	if (!SMP::goalFound)
		return false;

	std::list<Nodes*> tempPath = currPath;
	tempPath.reverse();
	Nodes* curr = tempPath.front();
	while (curr->parent != NULL)
	{
		if (curr->parent->costToStart == inf)
		{
			return false;
		}
		curr = curr->parent;
	}
	return true;
}

