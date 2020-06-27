    /**
 * @file
 * @brief Header file with class to transform point clouds
 */

#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "typedefs.h"
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

typedef Eigen::Matrix4f Matrix_t;
typedef float RealNumber_t;

/**
 * @brief Class which contains methods to transform point cloud
 */
class Transformation
{
public:

    /**
     * @brief Rotate point cloud around X axis
     * 
     * @param[in] cloud input point cloud
     * @param[in] angleRad angle of rotation around X axis in radians
     * 
     * @return rotated point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr rotateX(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t angleRad);

    /**
     * @brief Rotate point cloud around Y axis
     * 
     * @param[in] cloud input point cloud
     * @param[in] angleRad angle of rotation around Y axis in radians
     * 
     * @return rotated point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr rotateY(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t angleRad);

    /**
     * @brief Rotate point cloud around Z axis
     * 
     * @param[in] cloud input point cloud
     * @param[in] angleRad angle of rotation around Z axis in radians
     * 
     * @return rotated point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr rotateZ(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t angleRad);

    /**
     * @brief Move point cloud along X axis
     * 
     * @param[in] cloud input point cloud
     * @param[in] shift shift value
     * 
     * @return shifted point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr translateX(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shift);

    /**
     * @brief Move point cloud along Y axis
     * 
     * @param[in] cloud input point cloud
     * @param[in] shift shift value
     * 
     * @return shifted point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr translateY(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shift);

    /**
     * @brief Move point cloud along Z axis
     * 
     * @param[in] cloud input point cloud
     * @param[in] shift shift value
     * 
     * @return shifted point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr translateZ(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shift);

    /**
     * @brief Move point cloud along all three axises
     * 
     * @param[in] cloud input point cloud
     * @param[in] shiftX, shiftY, shiftZ shifts values along axises
     * 
     * @return shifted point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr translate(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shiftX,
		RealNumber_t shiftY, RealNumber_t shiftZ);
	// static pcl::PointCloud<Point_t>::Ptr translate(pcl::PointCloud<Point_t>::Ptr &cloud, Eigen::Vector3f shift);
    
    /**
     * @brief Find center of mass
     * 
     * @param[in] cloud input point cloud
     * @return mass center of input point cloud
     */
	static Eigen::Vector3f getMassCenter(pcl::PointCloud<Point_t>::Ptr &cloud);

    /**
     * @brief Find matrix of rotation around X
     * 
     * @param[in] angleRad angle of rotation around X axis in radians
     * @return rotation matrix around X
     */
    static Matrix_t getRotationX(RealNumber_t angleRad);

    /**
     * @brief Find matrix of rotation around Y
     * 
     * @param[in] angleRad angle of rotation around Y axis in radians
     * @return rotation matrix around Y
     */
    static Matrix_t getRotationY(RealNumber_t angleRad);

    /**
     * @brief Find matrix of rotation around Z
     * 
     * @param[in] angleRad angle of rotation around Z axis in radians
     * @return rotation matrix around Z
     */
    static Matrix_t getRotationZ(RealNumber_t angleRad);

    /**
     * @brief Rotate vector around X axis
     * 
     * @param[in] point input vector
     * @param[in] angleRad angle of rotation around X axis in radians
     * 
     * @return rotated vector
     */
    static Point_t rotateVectorX(Point_t &point, RealNumber_t angleRad);

    /**
     * @brief Rotate vector around Y axis
     * 
     * @param[in] point input vector
     * @param[in] angleRad angle of rotation around Y axis in radians
     * 
     * @return rotated vector
     */
    static Point_t rotateVectorY(Point_t &point, RealNumber_t angleRad);

    /**
     * @brief Rotate vector around Z axis
     * 
     * @param[in] point input vector
     * @param[in] angleRad angle of rotation around Z axis in radians
     * 
     * @return rotated vector
     */
    static Point_t rotateVectorZ(Point_t &point, RealNumber_t angleRad);
private:

    /**
     * @brief Transform point cloud
     * 
     * @param[in] cloud input point cloud
     * @param[in] transformation input transformation
     * 
     * @return transformed point cloud
     */
	static pcl::PointCloud<Point_t>::Ptr _transformCloud(pcl::PointCloud<Point_t>::Ptr &cloud, Matrix_t &transformation);
};

pcl::PointCloud<Point_t>::Ptr Transformation::rotateX(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t angleRad)
{
    Matrix_t transformation = getRotationX(angleRad);
    return _transformCloud(cloud, transformation);
}

pcl::PointCloud<Point_t>::Ptr Transformation::rotateY(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t angleRad)
{
    Matrix_t transformation = getRotationY(angleRad);
    return _transformCloud(cloud, transformation);
}

pcl::PointCloud<Point_t>::Ptr Transformation::rotateZ(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t angleRad)
{
    Matrix_t transformation = getRotationZ(angleRad);
    return _transformCloud(cloud, transformation);
}

Matrix_t Transformation::getRotationX(RealNumber_t angleRad)
{
    Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(1, 1) = cos(angleRad);
    transformation(1, 2) = -sin(angleRad);
    transformation(2, 1) = sin(angleRad);
    transformation(2, 2) = cos(angleRad);
    return transformation;
}

Matrix_t Transformation::getRotationY(RealNumber_t angleRad)
{
    Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(0, 0) = cos(angleRad);
    transformation(2, 0) = -sin(angleRad);
    transformation(0, 2) = sin(angleRad);
    transformation(2, 2) = cos(angleRad);
    return transformation;
}

Matrix_t Transformation::getRotationZ(RealNumber_t angleRad)
{
    Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(0, 0) = cos(angleRad);
    transformation(0, 1) = -sin(angleRad);
    transformation(1, 0) = sin(angleRad);
    transformation(1, 1) = cos(angleRad);
    return transformation;
}

pcl::PointCloud<Point_t>::Ptr Transformation::translateX(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shift)
{
    Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(0, 3) = shift;
    return _transformCloud(cloud, transformation);
}

pcl::PointCloud<Point_t>::Ptr Transformation::translateY(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shift)
{
    Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(1, 3) = shift;
    return _transformCloud(cloud, transformation);
}

pcl::PointCloud<Point_t>::Ptr Transformation::translateZ(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shift)
{
    Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(2, 3) = shift;
    return _transformCloud(cloud, transformation);
}

pcl::PointCloud<Point_t>::Ptr Transformation::translate(pcl::PointCloud<Point_t>::Ptr &cloud, RealNumber_t shiftX,
		RealNumber_t shiftY, RealNumber_t shiftZ)
{
	Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(0, 3) = shiftX;
    transformation(1, 3) = shiftY;
    transformation(2, 3) = shiftZ;
    return _transformCloud(cloud, transformation);
}

/*pcl::PointCloud<Point_t>::Ptr Transformation::translate(pcl::PointCloud<Point_t>::Ptr &cloud, Eigen::Vector3f shift)
{
	Matrix_t transformation = Eigen::Matrix4f::Identity();
    transformation(0, 3) = shift(0);
    transformation(1, 3) = shift(1);
    transformation(2, 3) = shift(2);
    return _transformCloud(cloud, transformation);
}*/

Eigen::Vector3f Transformation::getMassCenter(pcl::PointCloud<Point_t>::Ptr &cloud)
{
	pcl::MomentOfInertiaEstimation <Point_t> featureExtractor;
	featureExtractor.setInputCloud(cloud);
	featureExtractor.compute();
	Eigen::Vector3f massCenter;
	featureExtractor.getMassCenter(massCenter);
	return massCenter;
}


Point_t Transformation::rotateVectorX(Point_t &point, RealNumber_t angleRad)
{
    Matrix_t transformation = getRotationX(angleRad);
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);

    Eigen::Vector3f p(point.x, point.y, point.z);
    p = rotation * p;

    Point_t rotatedPoint = point;
    rotatedPoint.x = p(0);
    rotatedPoint.y = p(1);
    rotatedPoint.z = p(2);

    return rotatedPoint;
}

Point_t Transformation::rotateVectorY(Point_t &point, RealNumber_t angleRad)
{
    Matrix_t transformation = getRotationY(angleRad);
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);

    Eigen::Vector3f p(point.x, point.y, point.z);
    p = rotation * p ;

    Point_t rotatedPoint = point;
    rotatedPoint.x = p(0);
    rotatedPoint.y = p(1);
    rotatedPoint.z = p(2);

    return rotatedPoint;
}


Point_t Transformation::rotateVectorZ(Point_t &point, RealNumber_t angleRad)
{
    Matrix_t transformation = getRotationZ(angleRad);
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);

    Eigen::Vector3f p(point.x, point.y, point.z);
    p = rotation * p ;

    Point_t rotatedPoint = point;
    rotatedPoint.x = p(0);
    rotatedPoint.y = p(1);
    rotatedPoint.z = p(2);

    return rotatedPoint;
    }


pcl::PointCloud<Point_t>::Ptr Transformation::_transformCloud(pcl::PointCloud<Point_t>::Ptr &cloud, Matrix_t &transformation)
{
    pcl::PointCloud<Point_t>::Ptr cloudOut (new pcl::PointCloud<Point_t>);
    transformPointCloud(*cloud, *cloudOut, transformation);
    return cloudOut;	
}

#endif