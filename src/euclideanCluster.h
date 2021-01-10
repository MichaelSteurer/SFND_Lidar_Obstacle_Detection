#ifndef EULIDEANCLUSTER_H
#define EULIDEANCLUSTER_H


#include <vector>
#include "kdtree3d.h"

struct EuclideanClusterCustom
{

    bool findInList(int point, std::vector<int> *points)
    {
        return std::find(points->begin(), points->end(), point) != points->end();
    } 

    void proximity(const std::vector<std::vector<float>>& points, int index, KdTree* tree, float distanceTol, pcl::PointIndices::Ptr cluster, std::vector<int> *processedPointIndexes) 
    {
        processedPointIndexes->push_back(index);
        cluster->indices.push_back(index);

        std::vector<float> point = points[index];
        std::vector<int> nearbyPointIndexes = tree->search(point, distanceTol);
        for(int i = 0; i < nearbyPointIndexes.size(); i++)
        {
            int nearbyPointIndex = nearbyPointIndexes[i];
            if(!findInList(nearbyPointIndex, processedPointIndexes))
            {
                std::vector<float> nearbyPoint = points[nearbyPointIndex];
                proximity(points, nearbyPointIndex, tree, distanceTol, cluster, processedPointIndexes);
            }
        }
    }

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
    {
        std::vector<std::vector<int>> clusters;

        std::vector<int> processedPointIndexes;
        for(int pointsIndex = 0; pointsIndex < points.size(); pointsIndex++) 
        {
            if (!findInList(pointsIndex, &processedPointIndexes))
            {
                // std::cout << "processedPointIndexes:  " << std::endl;
                // for(int i = 0; i < processedPointIndexes.size(); i++)
                // {
                // 	std::cout << "  " << processedPointIndexes[i] << std::endl;
                // }
                pcl::PointIndices::Ptr cluster(new pcl::PointIndices());

                std::vector<float> point = points[pointsIndex];
                proximity(points, pointsIndex, tree, distanceTol, cluster, &processedPointIndexes);
                clusters.push_back(cluster->indices);
            }
        }
        return clusters;
    }
};
#endif