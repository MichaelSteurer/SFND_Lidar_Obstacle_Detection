#ifndef KDTREE3D_H
#define KDTREE3D_H


/* \author Aaron Brown */
// Quiz on implementing kd tree

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
		Node **node = findLeafToInsertRecursively(&root, point, 0);
		*node = new Node(point, id);
	}

	Node** findLeafToInsertRecursively(Node **node, std::vector<float> point, int depth)
	{
		if (*node == NULL) // termination condition
		{
			return node;
		}
		else 
		{
			Node **nextNode;

			int axisSelector = depth % 3; // 0: x, 1: y, 2: z, 3: x, 4: y, ...

			if ((*node)->point[axisSelector] < point[axisSelector])
			{
				nextNode = &((*node)->right);
			}
			else
			{
				nextNode = &((*node)->left);
			}			

			return findLeafToInsertRecursively(nextNode, point, depth + 1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		findNearbyNodesRecursively(&root, target, distanceTol, 0, &ids);
		return ids;
	}

	void findNearbyNodesRecursively(Node **node, std::vector<float> target, float distanceTol, int depth, std::vector<int> *foundIds)
	{		
		if (*node == NULL) // termination condition
		{
			// std::cout << "findNearbyNodesRecursively: Node is NULL" << std::endl;
			return;
		}

		// std::cout << "findNearbyNodesRecursively: Node " << (*node)->point[0] << "/" << (*node)->point[1] << std::endl;
		int axisSelector = depth % 3; // 0: x, 1: y, 2: z, 3: x, 4: y, ...

		float distanceLinePoint = (*node)->point[axisSelector] - target[axisSelector];
		bool lineInsideBox = abs(distanceLinePoint) < distanceTol;
		if (lineInsideBox)
		{
			bool pointInsideRadius = isPointInsideSphere(target, (*node)->point, distanceTol);
			if (pointInsideRadius)
			{
				(*foundIds).push_back((*node)->id);
			}
		}

		float lowerLimitBox = target[axisSelector] - distanceTol;
		if((*node)->point[axisSelector] > lowerLimitBox)
		{
			findNearbyNodesRecursively(&((*node)->left), target, distanceTol, depth + 1, foundIds);
		}

		float upperLimitBox = target[axisSelector] + distanceTol;
		if((*node)->point[axisSelector] < upperLimitBox)
		{
			findNearbyNodesRecursively(&((*node)->right), target, distanceTol, depth + 1, foundIds);
		}
	}


	bool isPointInsideSphere(std::vector<float> point, std::vector<float> sphereCenter, float distanceTol)
	{
		// get lower and upper bounds of the square box
		float sphereMaxX = sphereCenter[0] + distanceTol;
		float sphereMinX = sphereCenter[0] - distanceTol;
		float sphereMaxY = sphereCenter[1] + distanceTol;
		float sphereMinY = sphereCenter[1] - distanceTol;
		float sphereMaxZ = sphereCenter[2] + distanceTol;
		float sphereMinZ = sphereCenter[2] - distanceTol;

		// quick pre-check wether the point is outside this square box
		if (point[0] < sphereMinX || point[0] > sphereMaxX ||
			point[1] < sphereMinY || point[1] > sphereMaxY ||
			point[2] < sphereMinZ || point[2] > sphereMaxZ)
		{
			return false; 
		}

		// point is within the square box but not necssarily within the radius

		float deltaX = abs(point[0] - sphereCenter[0]);
		float deltaY = abs(point[1] - sphereCenter[1]);
		float deltaZ = abs(point[2] - sphereCenter[2]);
		float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

		return distance < distanceTol;
	}
};

#endif