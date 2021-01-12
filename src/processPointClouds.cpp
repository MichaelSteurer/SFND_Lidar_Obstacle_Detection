// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree3d.h"
#include "euclideanCluster.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    /////////////////////////////////////////////////
    // Voxel grid filter

    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFilteredGrid(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> filterGrid;
    filterGrid.setInputCloud(cloud);
    filterGrid.setLeafSize(filterRes, filterRes, filterRes);
    filterGrid.filter(*cloudFilteredGrid);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "grid filter took " << elapsedTime.count() << " milliseconds" << std::endl;


    /////////////////////////////////////////////////
    // Region filter

    startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFilteredGridRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> filterRegion(true);
    filterRegion.setInputCloud(cloudFilteredGrid);
    filterRegion.setMin(minPoint);
    filterRegion.setMax(maxPoint);
    filterRegion.filter(*cloudFilteredGridRegion);

    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "region filter took " << elapsedTime.count() << " milliseconds" << std::endl;


    /////////////////////////////////////////////////
    // Vehicle filter

    startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFilteredGridRegionRoof(new pcl::PointCloud<PointT>);

    std::vector<int> roofTopIndexes;

    // filter roof indexes
    pcl::CropBox<PointT> filterVehicle(true);
    filterVehicle.setInputCloud(cloudFilteredGridRegion);

    Eigen::Vector4f minCarPoint = Eigen::Vector4f (-1.5, -1.7, -1, 1);
    filterVehicle.setMin(minCarPoint);

    Eigen::Vector4f maxCarPoint = Eigen::Vector4f (2.6, 1.7, -0.4, 1);
    filterVehicle.setMax(maxCarPoint);
    filterVehicle.filter(roofTopIndexes);

    // remove these indexes from the already filtered cloud
    pcl::PointIndices::Ptr roofTopIndexesPcl(new pcl::PointIndices);
    for(int i = 0; i < roofTopIndexes.size(); i++)
    {
        int roofTopIndex = roofTopIndexes[i];
        roofTopIndexesPcl->indices.push_back(roofTopIndex);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudFilteredGridRegion);
    extract.setIndices(roofTopIndexesPcl);
    extract.setNegative(true);
    extract.filter(*cloudFilteredGridRegionRoof);

    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "roof filter took " << elapsedTime.count() << " milliseconds" << std::endl;


    // return result of grid-, region- and roof filter
    return cloudFilteredGridRegionRoof;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    pcl::PointCloud<PointT> *cloud_f = new pcl::PointCloud<PointT>();
    pcl::PointCloud<PointT> *cloud_p = new pcl::PointCloud<PointT>();

    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f);
    std::cerr << "PointCloud representing the obstacle component: " << cloud_f->width * cloud_f->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	srand(time(NULL));

    pcl::PointIndices::Ptr inliersGlobal(new pcl::PointIndices());

	for(int iteration=0; iteration < maxIterations; iteration++) 
	{
		// define three random distinct points in the cloud

		// first
		int randomIndexOne = rand() % cloud->size();
		PointT randomPointOne = cloud->points[randomIndexOne];

		// second
		int randomIndexTwo;
		do {
			randomIndexTwo = rand() % cloud->size();
		} 
		while(randomIndexTwo == randomIndexOne);

		PointT randomPointTwo = cloud->points[randomIndexTwo];

		// third
		int randomIndexThree;
		do {
			randomIndexThree = rand() % cloud->size();
		} 
		while(randomIndexThree == randomIndexOne || randomIndexThree == randomIndexTwo);

		PointT randomPointThree = cloud->points[randomIndexThree];

		// calculate A, B, C and D according to the equation Ax + By + Cz + D = 0
		float a = (randomPointTwo.y - randomPointOne.y) * (randomPointThree.z - randomPointOne.y) - (randomPointTwo.z - randomPointOne.z) * (randomPointThree.y - randomPointOne.y);
		float b = (randomPointTwo.z - randomPointOne.z) * (randomPointThree.x - randomPointOne.x) - (randomPointTwo.x - randomPointOne.x) * (randomPointThree.z - randomPointOne.z);
		float c = (randomPointTwo.x - randomPointOne.x) * (randomPointThree.y - randomPointOne.y) - (randomPointTwo.y - randomPointOne.y) * (randomPointThree.x - randomPointOne.x);
		float d = -(a * randomPointOne.x + b * randomPointOne.y + c * randomPointOne.z);

		// iterate through points, check if distance is within the threshold, i.e. it is an inlier.
		// if the number of inliers is larger than the current max -> set as new optimum.
        pcl::PointIndices::Ptr inliersIterationStep(new pcl::PointIndices());

		for(int i = 0; i < cloud->size(); i++)
		{
			// compute distance of current point
			PointT cloudPoint = cloud->points[i];
			float distance = abs(a * cloudPoint.x + b * cloudPoint.y + c * cloudPoint.z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

			// keep index if distance is less than threshold; this adds the three random points as well.
			if(distance < distanceTol)
			{ 
                inliersIterationStep->indices.push_back(i);
			}
		}

		// number of indexes is larger then current optimum -> store in global variable
		if(inliersIterationStep->indices.size() > inliersGlobal->indices.size())
		{
			inliersGlobal = inliersIterationStep;
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation custom took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersGlobal, cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation library took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> clustersIndexes;

    ///////////////////////////////////////////////////
    //// Use PCL KdTree Algorithm
    //// Creating the KdTree object for the search method of the extraction
    //
    //typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    //tree->setInputCloud(cloud);
    //
    //pcl::EuclideanClusterExtraction<PointT> ec;
    //ec.setClusterTolerance(clusterTolerance);
    //ec.setMinClusterSize(minSize);
    //ec.setMaxClusterSize(maxSize);
    //ec.setSearchMethod(tree);
    //ec.setInputCloud(cloud);
    //ec.extract(clustersIndexes);
    //
    //// Use PCL KdTree Algorithm
    ///////////////////////////////////////////////////

    /////////////////////////////////////////////////
    // Use Custom KdTree Algorithm

   	KdTree* tree = new KdTree;

    std::vector<std::vector<float>> points;    
    for(int i = 0; i < cloud->size(); i++)
    {
        PointT cloudPoint = cloud->points[i];
        std::vector<float> p{cloudPoint.x, cloudPoint.y, cloudPoint.z};
        points.push_back(p);
    	tree->insert(p, i); 
    }

    EuclideanClusterCustom* ecc = new EuclideanClusterCustom;
  	clustersIndexes = ecc->euclideanCluster(points, tree, clusterTolerance);

    // Use Custom KdTree Algorithm
    /////////////////////////////////////////////////

    for(int clusterIndexesIndex = 0; clusterIndexesIndex < clustersIndexes.size(); clusterIndexesIndex++) 
    {
        pcl::PointIndices clusterIndexes = clustersIndexes[clusterIndexesIndex];
        
        // apply size limits
        if (clusterIndexes.indices.size() > maxSize || clusterIndexes.indices.size() < minSize)
        {
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        // std::cout << "Cluster Index " << clusterIndexes.size() << std::endl;
        for(int clusterIndexesIndex = 0; clusterIndexesIndex < clusterIndexes.indices.size(); clusterIndexesIndex++) 
        {
            int clusterIndex = clusterIndexes.indices[clusterIndexesIndex];
            // std::cout << "  Index " << clusterIndex << std::endl;
            cluster->push_back(cloud->points[clusterIndex]);
        }
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}