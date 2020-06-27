/**
 * @file
 * @brief Header file with functions to filter clouds
 */

#ifndef FILTERS_H
#define FILTERS_H

#include <typedefs.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> //Уменьшение количества точек в облаке
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "transformations.h"


/**
 * @brief Remove points with depth values that are too large or too small
 * 
 * @param[in] cloud input point cloud
 * @param[in] minDepth minimum z value
 * @param[in] maxDepth maximum z value
 * 
 * @return Point cloud without points with z value out of range [minDepth, maxDepth]
 */
pcl::PointCloud<Point_t>::Ptr thresholdDepth (pcl::PointCloud<Point_t>::Ptr &cloud, float minDepth, float maxDepth)
{
    pcl::PassThrough<Point_t> pass_through;
    pass_through.setInputCloud (cloud);
    pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (minDepth, maxDepth);
    pcl::PointCloud<Point_t>::Ptr thresholded (new pcl::PointCloud<Point_t>);
    pass_through.filter (*thresholded);

   
 return (thresholded);
}

/**
 * @brief Calculate Euclidean norm of vector
 * 
 * @param[in] point input vector
 * @return Euclidean norm of vector (its length)
 */
float getEuclideanNorm(Point_t &point)
{
    return sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
}

/**
 * @brief Project point to xOy plane
 * 
 * @param[in] point input point
 * @return projected point
 */
Point_t getProjectionXY(Point_t &point)
{
    Point_t projection = point;
    projection.z = 0;
    return projection;
}

/**
 * @brief Align plane's normal with Z axis using rigid transformations
 * 
 * @param[in, out] cloud input point cloud which contains plane
 * @param[in] coefficients plane's coefficients
 * @param[in] planePoint any point of the plane
 */
void alignNormalWithZ(pcl::PointCloud<Point_t>::Ptr &cloud, pcl::ModelCoefficients::Ptr &coefficients, Point_t &planePoint)
{

    Point_t planeNormal;
    planeNormal.x = coefficients->values[0];
    planeNormal.y = coefficients->values[1];
    planeNormal.z = coefficients->values[2];

    //Translating point cloud to plane's point
    cloud = Transformation::translate(cloud, -planePoint.x, -planePoint.y, -planePoint.z);

    // Rotation to xOz plane
    Point_t normalProjectionXY = getProjectionXY(planeNormal);
    float angleZ = acos(planeNormal.x/ getEuclideanNorm(normalProjectionXY));

    pcl::PointCloud<Point_t>::Ptr transformedCloud(new pcl::PointCloud<Point_t>);

    cloud = Transformation::rotateZ(cloud, -angleZ);
    planeNormal = Transformation::rotateVectorZ(planeNormal, -angleZ);

    //Aligning with Z axis 
    float angleY = acos(planeNormal.z/getEuclideanNorm(planeNormal));
    cloud = Transformation::rotateY(cloud, -angleY);
}

/**
 * @brief Detect plane in the point cloud
 * 
 * @param[in] cloud input point cloud
 * @param[out] inliers indices of plane's points
 * @param[in] distanceThreshold [description]
 * 
 * @return pointer to plane's coefficients
 */
pcl::ModelCoefficients::Ptr detectPlane(pcl::PointCloud<Point_t>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, float distanceThreshold = 10.0)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<Point_t> seg; // Create the segmentation object
    seg.setOptimizeCoefficients (true); // Optional
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    return coefficients;
}

/**
 * @brief Remove plane from the point cloud and all points above it
 * 
 * @todo Add arguements
 * 
 * @param[in] cloud input point cloud
 * @return pointer to point cloud without plane and all points above it
 */
pcl::PointCloud<Point_t>::Ptr removePlane(pcl::PointCloud<Point_t>::Ptr &cloud)
{
    pcl::PointCloud<Point_t>::Ptr segmentedCloud(new pcl::PointCloud<Point_t>);
    *segmentedCloud = *cloud;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //Plane detection
    coefficients = detectPlane(segmentedCloud, inliers); // works good only after downsampling

    Point_t &planePoint = segmentedCloud->points[inliers->indices[5]];
    alignNormalWithZ(segmentedCloud, coefficients, planePoint);

    segmentedCloud = thresholdDepth(segmentedCloud, -10000000.0, -4.0);//-5 - works
    return segmentedCloud;
}


/**
 * @brief Reduce the number of points
 * 
 * @param[in] cloud input point cloud
 * @param[in] leafSize size of a cube in which all points will be replaced by one
 * 
 * @return downsampled point cloud
 */
pcl::PointCloud<Point_t>::Ptr downsample (pcl::PointCloud<Point_t>::Ptr &cloud, float leafSize)
{
    pcl::VoxelGrid<Point_t> voxelGrid;
    voxelGrid.setInputCloud (cloud);
    voxelGrid.setLeafSize (leafSize, leafSize, leafSize);
    pcl::PointCloud<Point_t>::Ptr downsampled (new pcl::PointCloud<Point_t>);
    voxelGrid.filter (*downsampled);
    return (downsampled);
}

/**
 * @brief Remove noisy measurements (outliers) from a point cloud using statistical analysis techniques
 * 
 * @param[in] cloud input point cloud
 * @param[in] k number of neighbors to analyze for each point, default is 50
 * @param[in] sdm standard deviation multiplier, default is 1.0
 * 
 * @return point cloud with removed outliers
 */
pcl::PointCloud<Point_t>::Ptr removeOutliersStatistically(pcl::PointCloud<Point_t>::Ptr &cloud, int k = 50, float sdm = 1.0)
{
    pcl::PointCloud<Point_t>::Ptr filteredCloud (new pcl::PointCloud<Point_t>());
    pcl::StatisticalOutlierRemoval<Point_t> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (k);
    sor.setStddevMulThresh (sdm);
    sor.filter (*filteredCloud);
    return filteredCloud;
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
/*PointCloud<Point_t>::Ptr removeOutliers (PointCloud<Point_t>::Ptr &input, float radius, int minNeighbors)
{
    RadiusOutlierRemoval<Point_t> radiusOutlierRemoval;
    radiusOutlierRemoval.setInputCloud (input);
    radiusOutlierRemoval.setRadiusSearch (radius);
    radiusOutlierRemoval.setMinNeighborsInRadius (minNeighbors);
    PointCloud<Point_t>::Ptr inliers (new PointCloud<Point_t>);
    radiusOutlierRemoval.filter (*inliers);

    return (inliers);
}*/


/**
 * @brief Clusterisation using Euclidean distance mean
 * 
 * @param  [in] cloud input point cloud
 * @param  [in] clusterTolerance determines minimum distance between points that belong to the same cluster
 * @param  [in] minClusterSize  defines minimum amount of points on cluster
 * @return set of clusters extracted from input cloud
 */
std::vector<pcl::PointCloud<Point_t>::Ptr> getEuclideanClusters(pcl::PointCloud<Point_t>::Ptr &cloud, 
    int clusterTolerance, int minClusterSize)
{
    pcl::search::KdTree<Point_t>::Ptr tree (new pcl::search::KdTree<Point_t>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<Point_t> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (cloud->points.size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    std::vector<pcl::PointCloud<Point_t>::Ptr> clusteredClouds;

    
    for(int i = 0; i < clusterIndices.size(); ++i)
    {
        pcl::PointCloud<Point_t>::Ptr ccloud(new pcl::PointCloud<Point_t>);
        for (int j = 0; j < clusterIndices[i].indices.size(); ++j)
            ccloud->points.push_back(cloud->points[clusterIndices[i].indices[j]]);
        clusteredClouds.push_back(ccloud);
        //ccloud->clear();
    }
    return clusteredClouds;
}




#endif
