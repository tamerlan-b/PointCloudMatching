/**
 * @file
 * @brief Header file with functions to compute descriptors
 */

#ifndef DESCRIPTORS_H
#define DESCRIPTORS_H

#include "typedefs.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h> 

#include <pcl/point_types_conversion.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>

/**
 * @brief Compute FPFH descriptors
 * @details FPFH - fast point feature histogram
 * 
 * @param[in] cloud input point cloud
 * @param[in] normals input surface normals
 * @param[in] keypoints input keypoints
 * @param[out] descriptors output descriptors
 * @param[in] k number of neighbors which used to compute descriptor, default is 3
 * @param[in] searchRadius search radius, to look for neighbors, it should be larger than the radius used to estimate the normals, default is 0.1f
 */
void computeFpfhDescriptors(pcl::PointCloud<Point_t>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, 
	pcl::PointCloud<Point_t>::Ptr &keypoints, pcl::PointCloud<Descriptor_t>::Ptr descriptors, int k = 3, float searchRadius = 0.1f)
{
    pcl::FPFHEstimation<Point_t, pcl::Normal, Descriptor_t> fpfh;
    fpfh.setSearchSurface(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(keypoints);
    fpfh.setSearchMethod(pcl::search::Search<Point_t>::Ptr (new pcl::search::KdTree<Point_t>));
    //fpfh.setKSearch(k);
    fpfh.setRadiusSearch(searchRadius);
    fpfh.compute(*descriptors);
}

/**
 * @brief Compute SHOT descriptors
 * @details SHOT - Signature of Histograms of Orientations
 * 
 * @param[in] cloud input point cloud
 * @param[in] normals input surface normals
 * @param[in] keypoints input keypoints
 * @param[out] descriptors output descriptors
 * @param radius defines which of the keypoint's neighbors are described, default is 10.0
 * 
 * Usage example:
 * @code
 * PointCloud<SHOT352>::Ptr descriptors (new PointCloud<SHOT352>);
 * computeShotDescriptors(cloud, normals, keypoints, descriptors, 5.0);
 * @endcode
 * 
 */
void computeShotDescriptors(pcl::PointCloud<Point_t>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, 
	pcl::PointCloud<Point_t>::Ptr &keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, float radius = 10.0f)
{
    pcl::SHOTEstimation<Point_t, pcl::Normal, pcl::SHOT352> shot;   ///< SHOT estimation object.
    shot.setSearchSurface(cloud);
    shot.setInputNormals(normals);
    shot.setInputCloud(keypoints);
    shot.setRadiusSearch(radius);
    // shot.setSearchMethod(pcl::search::Search<Point_t>::Ptr (new pcl::search::KdTree<Point_t>));
    // shot.setKSearch(5);
    shot.compute(*descriptors);
}

typedef pcl::Histogram<32> RIFT32;  ///< RIFT descriptors

/**
 * @brief Compute intensity gradients
 * 
 * @param[in] cloudIntensity input point cloud
 * @param[in] normals input surface normals
 * @param[out] gradients output gradients
 * @param[in] radiusSearch radius, to get all neighbors within, default is 0.03
 */
void computeIntensityGradients(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIntensity, pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::IntensityGradient>::Ptr &gradients, float radiusSearch = 0.03)
{
    pcl::IntensityGradientEstimation < pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient,
    pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
    ge.setInputCloud(cloudIntensity);
    ge.setInputNormals(normals);
    ge.setRadiusSearch(radiusSearch);
    ge.compute(*gradients);   
}

/**
 * @brief Compute RIFT descriptors
 * @details RIFT - Rotation-Invariant Feature Transform
 * 
 * @param[in] cloudColor input point cloud with RGB colors
 * @param[in] normals input surface normals
 * @param[out] descriptors output descriptors
 * @param radiusSearch radius, to get all neighbors within, default is 0.02
 * @param nrDistaceBins number of bins to use in the distance dimension, default is 4
 * @param nrGradientBins number of bins to use in the gradient orientation dimension, default is 8
 * 
 * Usage example:
 * @code
 * PointCloud<RIFT32>::Ptr descriptors(new pcl::PointCloud<RIFT32>());
 * computeRiftDescriptors(downsampledCloud, normals, descriptors, 0.05, 4, 8);
 * @endcode
 */
void computeRiftDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudColor, pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<RIFT32>::Ptr &descriptors, float radiusSearch = 0.02, int nrDistaceBins = 4, int nrGradientBins = 8)
{
    // Object for storing the point cloud with intensity value.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity(new pcl::PointCloud<pcl::PointXYZI>);
    // Object for storing the intensity gradients.
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
    // Convert the RGB to intensity.
    pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    computeIntensityGradients(cloudIntensity, normals, gradients);
    pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32> rift;   ///< RIFT estimation object.
    rift.setInputCloud(cloudIntensity);
    rift.setSearchMethod(kdtree);    
    rift.setInputGradient(gradients);   ///< Set the intensity gradients to use.
    rift.setRadiusSearch(radiusSearch);
    rift.setNrDistanceBins(nrDistaceBins);
    rift.setNrGradientBins(nrGradientBins);
    // Note: you must change the output histogram size to reflect the previous values.
    rift.compute(*descriptors);
}

#endif