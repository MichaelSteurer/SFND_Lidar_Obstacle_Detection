/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud;

    float resolution = 0.1;
    Eigen::Vector4f minPoint = Eigen::Vector4f (-10, -5, -2, 1);
    Eigen::Vector4f maxPoint = Eigen::Vector4f (25, 7, 3, 1);

    filteredCloud = pointProcessorI->FilterCloud(inputCloud, resolution, minPoint, maxPoint);

    // renderPointCloud(viewer, filteredCloud, "filterCloud");

    ProcessPointClouds<pcl::PointXYZI> *processPointClouds = new ProcessPointClouds<pcl::PointXYZI>();
    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> result;
    result = processPointClouds->SegmentPlaneCustom(filteredCloud, 1000, 0.2);

    // renderPointCloud(viewer, result.first, "first", Color(0, 255, 0));
    // renderPointCloud(viewer, result.second, "second", Color(255, 0, 0));

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(0,1,0)};
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processPointClouds->Clustering(result.first, 0.3, 50, 1000);
    for(int cloudClustersIndex = 0; cloudClustersIndex < cloudClusters.size(); cloudClustersIndex++)
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster = cloudClusters[cloudClustersIndex];
        processPointClouds->numPoints(cluster);

        Color color = colors[cloudClustersIndex % colors.size()];
        std::string name = "obstCloud" + std::to_string(cloudClustersIndex);

        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer, box, cloudClustersIndex);

        renderPointCloud(viewer, cluster, name, color);
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "my");
    
    ProcessPointClouds<pcl::PointXYZ> *processPointClouds = new ProcessPointClouds<pcl::PointXYZ>();
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> result;

	int segmentationAlgorithmSelector = 1;
	switch(segmentationAlgorithmSelector) 
	{
	case 0:
        result = processPointClouds->SegmentPlane(cloud, 1000, 0.2);
		break;
	case 1:
        result = processPointClouds->SegmentPlaneCustom(cloud, 1000, 0.2);
		break;
	default:
		return;
	}

    /////////////////////////////////////////////////
    // Render Segmentation Plane Objects

    // renderPointCloud(viewer, result.first, "first", Color(0, 255, 0));
    // renderPointCloud(viewer, result.second, "second", Color(255, 0, 0));

    // Render Segmentation Plane Objects
    /////////////////////////////////////////////////

    /////////////////////////////////////////////////
    // Cluster Identified Objects

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(0,1,0)};

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds->Clustering(result.first, 1, 4, 30);
    for(int cloudClustersIndex = 0; cloudClustersIndex < cloudClusters.size(); cloudClustersIndex++)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = cloudClusters[cloudClustersIndex];
        processPointClouds->numPoints(cluster);

        Color color = colors[cloudClustersIndex % colors.size()];
        std::string name = "obstCloud" + std::to_string(cloudClustersIndex);

        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer, box, cloudClustersIndex);

        renderPointCloud(viewer, cluster, name, color);
    }

    // Cluster Identified Objects
    /////////////////////////////////////////////////
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}