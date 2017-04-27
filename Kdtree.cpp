#include "Kdtree.h"
const int k = 2;
Data *insertRec(Data *root, Nodes node, unsigned depth)
{
	// Tree is empty?
	if (root == NULL)
	{
		Data *temp = new Data(node);
		return temp;
	}
	unsigned cd = depth % k;
	if (node.location[cd] < (root->node.location[cd]))
		root->left = insertRec(root->left, node, depth + 1);
	else
		root->right = insertRec(root->right, node, depth + 1);

	return root;
}
Data* insert(Data *root, Nodes node)
{
	return insertRec(root, node, 0);
}

bool arePointsSame(Nodes n1, Nodes n2)
{
	if (n1.location != n2.location) return false;
	return true;
}

bool searchRec(Data* root, Nodes node, unsigned depth)
{
	if (root == NULL)
		return false;
	if (arePointsSame(root->node, node))
		return true;
	unsigned cd = depth % k;

	if (node.location[cd] < root->node.location[cd])
		return searchRec(root->left, node, depth + 1);

	return searchRec(root->right, node, depth + 1);
}
bool search(Data* root, Nodes node)
{
	return searchRec(root, node, 0);
}


Nodes* nearSearchRec(Data* root, Nodes node, unsigned depth, std::pair <Nodes*, float> &bestNode)
{
	if (root == NULL)
		return bestNode.first;
	if (root->left != NULL) {
		float dist = root->left->node.location.distance(node.location);
		if (dist < bestNode.second) {
			bestNode.first = &(root->left->node);
			bestNode.second = dist;
		}
	}
	if (root->right != NULL) {
		float dist = root->right->node.location.distance(node.location);
		if (dist < bestNode.second) {
			bestNode.first = &(root->right->node);
			bestNode.second = dist;
		}
	}
	unsigned cd = depth % k;
	if (node.location[cd] < root->node.location[cd])
		return nearSearchRec(root->left, node, depth + 1, bestNode);

	return nearSearchRec(root->right, node, depth + 1, bestNode);
}
Nodes* nearSearch(Data* root, Nodes node)
{
	std::pair <Nodes*, float> bestNode;
	bestNode.first = &root->node;
	bestNode.second = bestNode.first->location.distance(node.location);
	return nearSearchRec(root, node, 0, bestNode);
}