/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		Node **node = findNodeRecursively(&root, point, 0);
		*node = new Node(point, id);
	}

	Node** findNodeRecursively(Node **node, std::vector<float> point, int depth)
	{
		if (*node == NULL) // termination condition
		{
			return node;
		}
		else 
		{
			Node **nextNode;
			if (depth % 2) // split y
			{
				if ((*node)->point[1] < point[1])
				{
					nextNode = &((*node)->right);
				}
				else
				{
					nextNode = &((*node)->left);
				}			
			} 
			else // split x
			{
				if ((*node)->point[0] < point[0])
				{
					nextNode = &((*node)->right);
				}
				else
				{
					nextNode = &((*node)->left);
				}
			}
			return findNodeRecursively(nextNode, point, depth++);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}


};




