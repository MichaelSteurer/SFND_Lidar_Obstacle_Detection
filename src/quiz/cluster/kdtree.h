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
		iterateNodesRecursively(&root, target, distanceTol, 0, &ids);
		return ids;
	}

	void iterateNodesRecursively(Node **node, std::vector<float> target, float distanceTol, int depth, std::vector<int> *foundIds)
	{		
		if (*node == NULL) // termination condition
		{
			// std::cout << "iterateNodesRecursively: Node is NULL" << std::endl;
			return;
		}

		// std::cout << "iterateNodesRecursively: Node " << (*node)->point[0] << "/" << (*node)->point[1] << std::endl;
		int pointIndex;
		if (depth % 2) // 1, 3, 5, ... check y
		{
			pointIndex = 1;
		}
		else // 0, 2, 4, ... check x
		{
			pointIndex = 0;
		}

		float distanceLinePoint = (*node)->point[pointIndex] - target[pointIndex];
		bool lineOutsideBox = abs(distanceLinePoint) > distanceTol;

		if (lineOutsideBox)
		{
			if (distanceLinePoint) // point is on the right side if value is positive
			{
				iterateNodesRecursively(&((*node)->right), target, distanceTol, depth + 1, foundIds);
			}
			else // point is on the left side
			{
				iterateNodesRecursively(&((*node)->left), target, distanceTol, depth + 1, foundIds);
			}
		}
		else // line is inside the current point's box
		{
			bool pointInsideRadius = isPointInsideRadius(target, (*node)->point, distanceTol);
			if (pointInsideRadius)
			{
				// std::cout << "iterateNodesRecursively: Node is a match " << (*node)->id << std::endl;
				(*foundIds).push_back((*node)->id);
			}

			iterateNodesRecursively(&((*node)->left), target, distanceTol, depth + 1, foundIds);
			iterateNodesRecursively(&((*node)->right), target, distanceTol, depth + 1, foundIds);
		}
	}


	bool isPointInsideRadius(std::vector<float> point, std::vector<float> boxCenter, float distanceTol)
	{
		// get lower and upper bounds of the square box
		float boxMaxX = boxCenter[0] + distanceTol;
		float boxMinX = boxCenter[0] - distanceTol;
		float boxMaxY = boxCenter[1] + distanceTol;
		float boxMinY = boxCenter[1] - distanceTol;

		// quick pre-check wether the point is outside this square box
		if (point[0] < boxMinX || point[0] > boxMaxX ||
			point[1] < boxMinY || point[1] > boxMaxY)
		{
			return false; 
		}

		// point is within the square box but not necssarily within the radius

		float deltaX = abs(point[0] - boxCenter[0]);
		float deltaY = abs(point[1] - boxCenter[1]);
		float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

		return distance < distanceTol;
	}
};




