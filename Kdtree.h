#pragma once
#include"nodeStruct.h"

struct Data {
	Nodes node;
	Data *left, *right;
	Data()
	{
		left = right = NULL;
	}
	Data(Nodes _node) {
		node = _node;
		left = right = NULL;
	}
};