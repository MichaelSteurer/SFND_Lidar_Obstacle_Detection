/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    std::unordered_set<int> indexInlinersGlobal;

	for(int iteration=0; iteration < maxIterations; iteration++) 
	{
		// define start and end of a random line
		int randomIndexStart = rand() % cloud->size();
		pcl::PointXYZ randomLineStart = cloud->points[randomIndexStart];

		int randomIndexEnd;
		do {
			randomIndexEnd = rand() % cloud->size();
		} 
		while(randomIndexEnd == randomIndexStart);

		pcl::PointXYZ randomLineEnd = cloud->points[randomIndexEnd];
		
		// calculate A, B and C according to the equation Ax + By + C = 0
		float a = randomLineStart.y - randomLineEnd.y;
		float b = randomLineEnd.x - randomLineStart.x;
		float c = randomLineStart.x * randomLineEnd.y - randomLineEnd.x * randomLineStart.y;

		// iterate through points, check if distance is within the threshold, i.e. it is an inlier.
		// if the number of inliers is larger than the current max -> set as new optimum.
	    std::unordered_set<int> indexInliersIterationStep;
		for(int i = 0; i < cloud->size(); i++)
		{
			// compute distance
			// taken from https://brilliant.org/wiki/dot-product-distance-between-point-and-a-line/
			pcl::PointXYZ cloudPoint = cloud->points[i];
			float d = abs(a * cloudPoint.x + b * cloudPoint.y + c) / sqrt(pow(a, 2) + pow(b, 2));

			// keep index if distance is less than threshold; adds start and end of line too.
			if(d < distanceTol)
			{ 
				indexInliersIterationStep.insert(i);
			}
		}

		// number of indexes is larger then current optimum -> store in global variable
		if(indexInliersIterationStep.size() > indexInlinersGlobal.size())
		{
			indexInlinersGlobal = indexInliersIterationStep;
		}
	}

	// return global optimum
	return indexInlinersGlobal;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
