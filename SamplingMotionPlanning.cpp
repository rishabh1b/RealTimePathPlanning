#include "SMP.h"
#include "RRTstar.h"
#include "InformedRRTstar.h"
#include "simulationParam.h"
#include "RT-RRTstar.h"

bool SMP::goalFound = false;
bool SMP::moveNow = false;
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

bool SMP::checkCollision(Nodes n1, Nodes n2, const list<obstacles> obst)
{
	ofVec2f temp = n2.location - n1.location;
	float m = temp.y / temp.x;
	float c = n1.location.y - m* n1.location.x;
	for (auto i : obst) {
		float dist = abs(m*i.loc().x - i.loc().y + c) / hypot(m, 1);
		if (dist < i.rad()) {
			return false;
		}
	}
	return true;
}



bool SMP::checkSample(Nodes n, const list<obstacles> obst)
{
	for (auto i : obst) {
		if (n.location.distance(i.loc()) <= i.rad()) return false;
	}
	return true;
}

void RRTstar::nextIter(std::list<Nodes>& nodes, const list<obstacles> obst, Nodes* u_)
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
	//if (closestNeighbours.empty()) {
	//	closestNeighbours.push_back(v);
	//}
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
	while (it != nodes.end())
	{
		if (u.location.distance(it->location) < rrtstarradius)
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
	return closestNeighbours;
}

void InformedRRTstar::nextIter(std::list<Nodes> &nodes, const std::list<obstacles> obst)
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
				min_cost = (*it)->costToStart;
			it++;
		}

		RRTstar::nextIter(nodes, obst, &InformedRRTstar::sample(min_cost));
	}
	if (SMP::goalFound)
		sol_nodes.push_back(&nodes.back());
}

Nodes InformedRRTstar::sample(float c_max)
{
	float c_min = SMP::goal.distance(SMP::start);

	if (std::abs(c_max - c_min) < 100) //Putting a dummy value for now - Robot might not move for some configurations with this value
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

void RTRRTstar::nextIter(std::list<Nodes> &nodes, const std::list<obstacles>& obst, Nodes* u_)
{
	timeKeeper = ofGetElapsedTimef();
	expandAndRewire(nodes, obst);
	updateNextBestPath();
	//if car.getPosition() close to x0(root), then-
	Nodes* nextPoint = *(currPath.begin()++); //Change the DS for path to vector?
	SMP::root->location.x = nextPoint->location.x;
	SMP::root->location.y = nextPoint->location.y;
}

void RTRRTstar::expandAndRewire(std::list<Nodes>& nodes, const std::list<obstacles>& obst)
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
		if (this->closestNeighbours.size() < maxNeighbours)
		{
			this->addNode(u, v, nodes, obst);
		}
		else
		{
			this->rewireRand.push(v);
		}
		rewireRandomNode(obst, nodes);
	}
	rewireFromRoot(obst, nodes);
}

void RTRRTstar::updateNextBestPath()
{
	// Will assign pathAvailable variable
	Nodes *pathNode = target;
	if (SMP::goalFound) {
		do
		{
			currPath.push_back(pathNode);
			pathNode = pathNode->parent;
		} while (pathNode->parent != NULL);
	}
	else {
		Nodes* curr_node = SMP::root;
		while (1)
		{
			//std::list<Nodes*>::iterator pathIT = currPath.begin();
			//std::list<Nodes*> updatedPath;
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
			}
			currPath.push_back(tempNode);
			if (tempNode->children.empty() || cost(tempNode) == inf)
			{
				visited_set.insert(tempNode);
				break;
			}
			curr_node = tempNode;
		} 
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
	else if (rand_num >= (1 - alpha) / beta && pathAvailable)
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
	while (it != nodes.end())
	{
		float dist = u.location.squareDistance((*it).location);
		if (dist < min_dist)
		{
			min_dist = dist;
			near_node = &(*it);
		}
		if (u.location.distance(it->location) < rrtstarradius) //TODO:Change this constant value
		{
			closestNeighbours.push_back(&(*it));
		}
		it++;
	}
	return near_node;
}

void RTRRTstar::addNode(Nodes n, Nodes* closest, std::list<Nodes>& nodes, const std::list<obstacles>& obst)
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
	}
	n.parent = parent;
	nodes.push_back(n);
	parent->children.push_back(&(nodes.back()));

	if (SMP::target != NULL && n.location.distance(target->location) < converge)
	{
		SMP::goalFound = true;
	}
	//TODO: Add the node to the Grid based/KD-Tree Data structure

	this->rewireRand.push(&(nodes.back()));
}

float RTRRTstar::cost(Nodes* node)
{
	float cost_ = 0;
	Nodes* curr = node;
	while (curr->parent != NULL)
	{
		if (curr->parent->costToStart == inf)
		{
			node->costToStart = inf;
			break;
		}
		cost_ += curr->location.distance(curr->parent->location);
		curr = curr->parent;
	}
	return cost_;
}


//std::list<Nodes*> RTRRTstar::findNodesNear(Nodes u)
//{
//
//}

void RTRRTstar::rewireRandomNode(const list<obstacles> obst, std::list<Nodes> &nodes)
{
	//std::list<Nodes*>::iterator mainIT = rewireRand.begin();
	while (!rewireRand.empty() && (ofGetElapsedTimef() - timeKeeper) < allowedTimeRewiring)
	{
		Nodes* Xr = rewireRand.front();
		rewireRand.pop();

		std::list<Nodes*> nearNodes = RRTstar::findClosestNeighbours(*Xr, nodes);
		std::list<Nodes*>::iterator it = nearNodes.begin();
		std::list<Nodes*> safeNeighbours;
		while (it != nearNodes.end())
		{
			if (SMP::checkCollision(*Xr, *(*it), obst))
				safeNeighbours.push_back(*it);
			it++;
		}
		if (safeNeighbours.empty()) return;

		it = safeNeighbours.begin();
		while (it != safeNeighbours.end())
		{

			float oldCost = cost(*it);
			float newCost = cost(Xr) + Xr->location.distance((*it)->location);
			if (newCost < oldCost)
			{

				(*it)->prevParent = (*it)->parent;
				(*it)->parent->children.remove(*it);
				(*it)->parent = Xr;
				(*it)->costToStart = newCost;
				Xr->children.push_back(*it);
				it++;
				rewireRand.push(*it);
			}
		}
	}
}

void RTRRTstar::rewireFromRoot(const list<obstacles> obst, std::list<Nodes> &nodes) {

	if (rewireRoot.empty()) {
		rewireRoot.push_back(SMP::root);
	}

	while (!rewireRoot.empty() || (ofGetElapsedTimef() - timeKeeper) < allowedTimeRewiring) {

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
		if (safeNeighbours.empty()) return;

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
				it++;
			}

			bool found = std::find(rewireRoot.begin(), rewireRoot.end(), (*it)) != rewireRoot.end();
			if (!found) {
				rewireRoot.push_back((*it));
			}
		}
	}
}

float RTRRTstar::getHeuristic(Nodes* u) {
	if (visited_set.find(u) != visited_set.end())
		return inf;
	else
		return u->location.distance(SMP::target->location);
}

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

