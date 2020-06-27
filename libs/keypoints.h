/**
 * @file
 * @brief Header file with functions to extrat keypoints
 */

#ifndef KEYPOINTS_H
#define KEYPOINTS_H

#include "typedefs.h"
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>

/**
 * @brief Detect set of SIFT keypoints
 * 
 * @param[in] cloud input point cloud
 * @param[in] normals input surface normals
 * @param[out] keypoints output keypoints without scale
 * @param[in] minScale the smallest scale in the difference-of-Gaussians (DoG) scale-space, default is 1.5
 * @param[in] nr_octaves the number of times the scale doubles in the DoG scale-space, default is 1
 * @param[in] nr_scales_per_octave the number of scales computed for each doubling, default is 2
 * @param[in] minContrast minimum local contrast that must be present for a keypoint to be detected, default is 0.0
 * 
 * @return pointer to a point cloud of keypoints with scale
 */
pcl::PointCloud<pcl::PointWithScale>::Ptr detectSiftKeypoints (pcl::PointCloud<Point_t>::Ptr & cloud, 
    pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<Point_t>::Ptr &keypoints, 
    float minScale = 1.5, int nr_octaves = 1, int nr_scales_per_octave = 2, float minContrast = 0.0)
{
    pcl::SIFTKeypoint<Point_t, pcl::PointWithScale> siftDetect;
    siftDetect.setSearchMethod (pcl::search::Search<Point_t>::Ptr (new pcl::search::KdTree<Point_t>));
    siftDetect.setScales (minScale, nr_octaves, nr_scales_per_octave);
    siftDetect.setMinimumContrast (minContrast);
    siftDetect.setInputCloud (cloud);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypointsTemp (new pcl::PointCloud<pcl::PointWithScale>);
    siftDetect.compute (*keypointsTemp);
    pcl::copyPointCloud (*keypointsTemp, *keypoints);
    return keypointsTemp;
}

/**
 * @todo Add description
 * @brief This function performs the spatial resolution computation for a given point cloud 
 * averaging the distance between each cloud point and its nearest neighbor.
 */
double computeCloudResolution(pcl::PointCloud<Point_t>::Ptr &cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<Point_t> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (! pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }   
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    return resolution;
}

/**
 * @brief Extract keypoints using ISS detector
 * @details ISS - Intrinsic Shape Signatures
 * 
 * @param[in] cloud input point cloud
 * @param[out] keypoints output keypoints
 * @param[in] minNeighbors minimum number of neighbors that has to be found while applying the non maxima suppression algorithm, default is 5
 * @param[in] threshold21 upper bound on the ratio between the second and the first eigenvalue, default is 0.975
 * @param[in] threshold32 upper bound on the ratio between the third and the second eigenvalue, default is 0.975
 * @param[in] numberOfThreads number of processing threads to use. 0 sets it to automatic, default is 4
 */
void detectIssKeypoints(pcl::PointCloud<Point_t>::Ptr &cloud, pcl::PointCloud<Point_t>::Ptr &keypoints, 
	int minNeighbors = 5, float threshold21 = 0.975, float threshold32 = 0.975, int numberOfThreads = 4)
{
    pcl::ISSKeypoint3D<Point_t, Point_t> detector;
    detector.setInputCloud(cloud);
    pcl::search::KdTree<Point_t>::Ptr kdtree(new pcl::search::KdTree<Point_t>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);    
    detector.setSalientRadius(6 * resolution);  ///< Set the radius of the spherical neighborhood used to compute the scatter matrix.
    detector.setNonMaxRadius(4 * resolution);   ///< Set the radius for the application of the non maxima supression algorithm.
    detector.setMinNeighbors(minNeighbors);
    detector.setThreshold21(threshold21);
    detector.setThreshold32(threshold32);
    detector.setNumberOfThreads(numberOfThreads);
    detector.compute(*keypoints);
}




#endif