

#ifndef MATCHING_H
#define MATCHING_H


#include "typedefs.h"
#include "filters.h"
#include "keypoints.h"
#include "descriptors.h"
#include "visualization.h"
#include "transformations.h"
#include <iostream>
#include <pcl/io/ply_io.h> //Work with .ply files
#include <pcl/io/pcd_io.h> //Work with .pcd files
#include "pcl/features/normal_3d.h" // Normals estimation
#include "pcl/search/kdtree.h" //kd search tree
#include <pcl/registration/ia_ransac.h> //initial alignment with ransac
#include <string> //using string class

//plane detection libraries
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/registration/sample_consensus_prerejective.h> //!!!!

#include <math.h>
#include <pcl/common/angles.h>

#include <pcl/registration/icp.h>
#include <vector>

using namespace std;
using namespace pcl;


#define DEBUG false


/*
Pipeline
loading
downsampling
remove ground
filtering
remove noise using clustering (connected components)
estimate normals
detect keypoints
extract descriptors
perform IA
refine with ICP
*/

/**
 * @brief Load point cloud from PLY file
 * 
 * @param[in] path path to .ply file with point cloud
 * @param[out] cloud output loaded point cloud
 * 
 * @return if loaded true, else - false
 */
bool loadPlyCloud(string path, PointCloud<Point_t>::Ptr &cloud)
{
    return io::loadPLYFile<Point_t>(path, *cloud) < 0 ? false : true;
}

/**
 * @brief Print point cloud in console
 * 
 * @param[in] cloud input point cloud
 * @param[in] numberOfPrintedPoints number of points to print, if 0 - prints all the points of point cloud
 */
void printPointCloud(PointCloud<Point_t>::Ptr &cloud, int numberOfPrintedPoints = 0)
{
    if(numberOfPrintedPoints == 0)
        numberOfPrintedPoints = cloud->points.size();
    for (int i = 0; i < numberOfPrintedPoints; i++) 
    {
        cout<< i<<". " 
            << cloud->points[i].x <<"; "
            <<cloud->points[i].y<<"; "
            <<cloud->points[i].z <<";"<<endl;
    }
}

/**
 * @brief Print transformation matrix in console
 * 
 * @param[in] transformation input transformation matrix
 */
void printTransformationMatrix(Eigen::Matrix4f &transformation)
{
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

    std::cout << "Transformation matrix:" << std::endl << std::endl;
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    std::cout << std::endl;
    printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));   
}

/**
 * @brief Visualize not aligned and aligned point clouds
 * 
 * @param[in] source input source cloud
 * @param[in] target input target cloud
 * @param[in] transformation transformation required to align source cloud to target
 * 
 */
void showAlignmentResult(PointCloud<Point_t>::Ptr &source, PointCloud<Point_t>::Ptr &target, Eigen::Matrix4f &transformation)
{
    PointCloud<Point_t>::Ptr transformed (new PointCloud<Point_t>());
    transformPointCloud(*source, *transformed, transformation);

    vector<pcl::PointCloud<Point_t>::Ptr > clouds;
    clouds.push_back(source);
    clouds.push_back(target);

    vector<pcl::PointCloud<Point_t>::Ptr > alignedClouds;
    alignedClouds.push_back(transformed);
    alignedClouds.push_back(target);

    vector<Color> colors;
    colors.push_back(Gold);
    colors.push_back(LightSeaGreen);

    showInTwoWindows(clouds, alignedClouds, colors, colors);
}

/**
 * @brief Visualize not aligned and aligned point clouds
 * 
 * @param[in] source input source cloud
 * @param[in] target input target cloud
 * @param[in] transformation transformation required to align source cloud to target
 * 
 */
void showAlignmentResult(PointCloud<Point_t>::Ptr &source, PointCloud<Point_t>::Ptr &target, 
    Eigen::Matrix4f &transformationIA, Eigen::Matrix4f &refineTransformation)
{
    // PointCloud<Point_t>::Ptr transformed (new PointCloud<Point_t>());
    // transformPointCloud(*source, *transformed, transformation);

    vector<pcl::PointCloud<Point_t>::Ptr > clouds;
    clouds.push_back(source);
    clouds.push_back(target);

    PointCloud<Point_t>::Ptr transformed (new PointCloud<Point_t>());
    transformPointCloud(*source, *transformed, transformationIA);
    transformPointCloud(*transformed, *transformed, refineTransformation);


    vector<pcl::PointCloud<Point_t>::Ptr > alignedClouds;
    alignedClouds.push_back(transformed);
    alignedClouds.push_back(target);

    vector<Color> colors;
    colors.push_back(Gold);
    colors.push_back(LightSeaGreen);

    showInTwoWindows(clouds, alignedClouds, colors, colors);
}


/**
 * @brief Estimate a cloud's surface normals
 * 
 * @todo Add arguements
 * 
 * @param[in] cloud input point cloud
 * @param[in] radius size of the local neighborhood used to estimate the surface
 * 
 * @return pointer to a SurfaceNormals point cloud
 */
PointCloud<Normal>::Ptr estimateSurfaceNormals (PointCloud<Point_t>::Ptr &cloud, float radius)
{
    NormalEstimation<Point_t, Normal> normalEstimation;
    normalEstimation.setSearchMethod (search::Search<Point_t>::Ptr (new search::KdTree<Point_t>));
    normalEstimation.setRadiusSearch (radius);
    normalEstimation.setInputCloud (cloud);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    normalEstimation.compute (*normals);
    return (normals);
}

/**
 * @brief Estimate a cloud's surface normals
 * 
 * @todo Add arguements
 * 
 * @param[in] cloud input point cloud
 * @param[in] k number of the local neighbors used to estimate the surface
 * 
 * @return pointer to a SurfaceNormals point cloud
 */
PointCloud<Normal>::Ptr estimateSurfaceNormals (PointCloud<Point_t>::Ptr &cloud, int k = 4)
{
    NormalEstimation<Point_t, Normal> normalEstimation;
    normalEstimation.setSearchMethod (search::Search<Point_t>::Ptr (new search::KdTree<Point_t>));
    normalEstimation.setKSearch(k);
    normalEstimation.setInputCloud (cloud);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    normalEstimation.compute (*normals);
    return (normals);
}


/**
 * @brief Load two point clouds and show them together painted in different colors
 * @details [long description]
 * 
 * @param[in] pathA, pathB paths to point cloud's .ply files
 */
void loadTwoCloudsAndShowThem(string pathA , string pathB)
{
    PointCloud<Point_t>::Ptr cloudA(new PointCloud<Point_t>);
    PointCloud<Point_t>::Ptr cloudB(new PointCloud<Point_t>);
    bool isLoadedA = loadPlyCloud(pathA, cloudA);
    bool isLoadedB = loadPlyCloud(pathB, cloudB);
    if(isLoadedA && isLoadedB)
    {
        vector<pcl::PointCloud<Point_t>::Ptr > clouds;
        clouds.push_back(cloudB);
        clouds.push_back(cloudA);

        vector<Color> colorsA;
        colorsA.push_back(Gold);
        colorsA.push_back(Purple);

        vector<Color> colorsB;
        colorsB.push_back(FireBrick);
        colorsB.push_back(LimeGreen);

        showInTwoWindows(clouds, clouds, colorsA, colorsB);
        //showInTwoWindows(cloudA, cloudB);
        //showPointClouds(cloudA, cloudB);
    
    }
}

/**
 * @brief Load downsamlped point cloud from .ply file
 * @details Load downsamlped point cloud, if
 * it doesn't exist, load full point cloud,
 * downsample it and save in file
 * 
 * @param[in] cloudPath path to full point cloud
 * @param[in] downsampledCloudPath path to downsampled point cloud
 * @param[out] downsampledCloud output point cloud
 * @param[in] leafSize size of a cube in which all points will be replaced by one, default is 4.0
 * 
 * @return if loaded cloud true, else false
 */
bool loadOrCreateDownsampledCloud(string cloudPath, string downsampledCloudPath, 
    PointCloud<Point_t>::Ptr &downsampledCloud, float leafSize = 4.0f)
{
    //Load downsampled point cloud
    bool isLoaded = loadPlyCloud(downsampledCloudPath, downsampledCloud);
    //If it hasn't loaded
    if(!isLoaded) 
    {
        //Load full point cloud
        PointCloud<Point_t>::Ptr cloud (new PointCloud<Point_t>());
        isLoaded = loadPlyCloud(cloudPath, cloud);
        if(isLoaded)
        {
            //Downsample full point cloud
            downsampledCloud = downsample(cloud, leafSize);
            //Save downsampled point cloud
            io::savePLYFileASCII(downsampledCloudPath, *downsampledCloud);
        }
    }
    return isLoaded;    
}

//---Initial alignment---

/**
 * @brief Align point clouds using SampleConsensusInitialAlignment
 * 
 * @param[in] sKeypoints source keypoints
 * @param[in] tKeypoints target keypoints
 * @param[in] sDescriptors source keypoints's descriptors
 * @param[in] tDescriptors target keypoints's descriptors
 * @param[in] n number of points from the source cloud
 * @param[in] d minimum distance between n points
 * @param[in] k_ia number of closest points to the source cloud from the target
 * @param[in] N max amount of iterations
 * @param[in] threshold [description]
 * 
 * @return transformation matrix (from source to target) to align point clouds
 */
Eigen::Matrix4f alignWithSacIa(PointCloud<Point_t>::Ptr &sKeypoints, PointCloud<Point_t>::Ptr &tKeypoints,
    PointCloud<Descriptor_t>::Ptr &sDescriptors, PointCloud<Descriptor_t>::Ptr &tDescriptors,
    int n = 100, float d = 0.5f, int k_ia = 7, int N = 5000, double threshold = 0.005)
{
    pcl::SampleConsensusInitialAlignment <Point_t, Point_t, Descriptor_t> sac_ia;
    if(n >= sKeypoints->size())
        n = sKeypoints->size() - 1;
    sac_ia.setNumberOfSamples (n);
    sac_ia.setMinSampleDistance (d);
    sac_ia.setCorrespondenceRandomness (k_ia);
    //sac_ia.setMaximumIterations (N);
    sac_ia.setRANSACOutlierRejectionThreshold (threshold);
    sac_ia.setInputSource(sKeypoints);
    sac_ia.setInputTarget (tKeypoints);
    sac_ia.setSourceFeatures (sDescriptors);
    sac_ia.setTargetFeatures (tDescriptors);
    PointCloud<Point_t> alignedSource;
    sac_ia.align (alignedSource);
    Eigen::Matrix4f transformation = sac_ia.getFinalTransformation ();

    if(DEBUG)
    {
        if (sac_ia.hasConverged())
        printTransformationMatrix(transformation);
        else std::cout << "Did not converge." << std::endl;
    }
    return transformation;
}

//------

/**
 * @brief Transit point cloud to mass center
 * 
 * @param[in, out] cloud input point cloud
 */
void transitToMassCenter(PointCloud<Point_t>::Ptr& cloud)
{
    Eigen::Vector3f massCenter = Transformation::getMassCenter(cloud);
    cloud = Transformation::translate(cloud, -massCenter(0), -massCenter(1), -massCenter(2));
}


PointCloud<Point_t>::Ptr concatenateClouds(vector<PointCloud<Point_t>::Ptr> &clouds)
{
	PointCloud<Point_t>::Ptr concatenatedCloud = clouds[0];
	for (int i = 1; i < clouds.size(); ++i)
	{
		*concatenatedCloud += *clouds[i];
	}
	return concatenatedCloud;
}


 void alignWithSacPrerejective(PointCloud<Point_t>::Ptr &sCloud, PointCloud<Point_t>::Ptr &tCloud,
    PointCloud<Point_t>::Ptr &sKeypoints, PointCloud<Point_t>::Ptr &tKeypoints,
    PointCloud<Descriptor_t>::Ptr &sDescriptors, PointCloud<Descriptor_t>::Ptr &tDescriptors)
{
    SampleConsensusPrerejective<Point_t,Point_t, Descriptor_t> align;
    align.setInputSource (sKeypoints);
    align.setSourceFeatures (sDescriptors);
    align.setInputTarget (tKeypoints);
    align.setTargetFeatures (tDescriptors);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (10); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * 4.0); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    
    PointCloud<Point_t>::Ptr object_aligned(new PointCloud<Point_t>);
    align.align (*object_aligned);
    

    if (align.hasConverged ())
    {
        // Print results
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), sCloud->size ());

        // Show alignment
        showAlignmentResult(sCloud, tCloud, transformation);
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        // return (1);
    }
}



/*void AlignWithIcp()
{
    //Transform source cloud
    // PointCloud<Point_t>::Ptr transformedSource(new PointCloud<Point_t>);
    // transformPointCloud(*sCloud, *transformedSource, transformation);

    //ICP
    // PointCloud<Point_t>::Ptr finalCloud(new PointCloud<Point_t>);
    // IterativeClosestPoint<Point_t, Point_t> registration;
    // registration.setInputSource(transformedSource);
    // registration.setInputTarget(tCloud);

    // registration.align(*finalCloud);
    // if (registration.hasConverged())
    // {
    //     cout << "ICP converged." << endl
    //           << "The score is " << registration.getFitnessScore() << endl;


    //     Eigen::Matrix4f transformationIcp = registration.getFinalTransformation();
    //     cout << "Transformation matrix:" <<endl<<transformationIcp<<endl;

    //     showAlignmentResult(transformedSource, tCloud, transformationIcp);

    // }
    // else cout << "ICP did not converge." << endl;
}*/



/**
 * @brief Registrate two point clouds
 */
void registrateClouds()
{
    //Here prefix (or suffix) 's' means 'source', 't' means 'target'
    
    //Paths to the full point clouds
    string sCloudPath = "../data/full_ascii/view1.ply";
    //string tCloudPath = "../data/full_ascii/view2cc.ply";
    string tCloudPath = "../data/full_ascii/view3.ply";
    // string tCloudPath = "../data/full_ascii/view4.ply";

    //Paths to the full cropped point clouds
    // string sCloudPath = "../data/cropped/view1ca.ply";
    // string tCloudPath = "../data/cropped/view2сa.ply";

    //---Parameters for functions---
    float leafSize = 4.0;//downsampling parameters
    float radius = 2 * leafSize; //normal estimation parameter
    // float radius = 5.0f;

    //Object for storing point clouds
    PointCloud<Point_t>::Ptr sCloud(new PointCloud<Point_t>);
    PointCloud<Point_t>::Ptr tCloud(new PointCloud<Point_t>);
    
    bool isLoadedS = loadPlyCloud(sCloudPath, sCloud);
    bool isLoadedT = loadPlyCloud(tCloudPath, tCloud);

    if(isLoadedS && isLoadedT)
    {
        
        //---Downsampling---
        sCloud = downsample(sCloud, leafSize);
        tCloud = downsample(tCloud, leafSize);

        //---Remove plane---
        sCloud = removePlane(sCloud);
        tCloud = removePlane(tCloud);
        //showInTwoWindows(sCloud, tCloud);


        //---Filtering cloud---
        sCloud = removeOutliersStatistically(sCloud, 50, 0);
        tCloud = removeOutliersStatistically(tCloud, 50, 0);
        // tCloud = removeOutliersStatistically(tCloud, 10, 5.0); //for cropped
        // showInTwoWindows(sCloud, tCloud);
        
        if(DEBUG)
        {
            cout<<"Amount of filtered source cloud points: "<<sCloud->points.size()<<endl;
            cout<<"Amount of filtered target cloud points: "<<tCloud->points.size()<<endl;
        }

        //---Transition to the mass center point---
        transitToMassCenter(sCloud);
        transitToMassCenter(tCloud);


        //---Reducing small clusters
        vector<PointCloud<Point_t>::Ptr> sClusters = getEuclideanClusters(sCloud, 15, 100);
        sCloud = concatenateClouds(sClusters);
        vector<PointCloud<Point_t>::Ptr> tClusters = getEuclideanClusters(tCloud, 15, 100);
        tCloud = concatenateClouds(tClusters);

        if(DEBUG)
        {
            cout<<"Amount of source points: "<<sCloud->points.size()<<endl;
            cout<<"Amount of target points: "<<tCloud->points.size()<<endl;
            // showPointClouds(sCloud, tCloud);
        }


        //---Estimating normals---
        if(DEBUG)
            cout<<"Estimating normals ...";

        PointCloud<Normal>::Ptr sNormals = estimateSurfaceNormals(sCloud, radius);
        PointCloud<Normal>::Ptr tNormals = estimateSurfaceNormals(tCloud, radius);

        if(DEBUG)
        {
            cout<<"OK"<<endl;
            cout<<"Amount of source normals: "<<sNormals->points.size()<<endl;
            cout<<"Amount of target normals: "<<tNormals->points.size()<<endl;
            cout<<"First source normal:"<<sNormals->points[0]<<endl;
            cout<<"First target normal:"<<tNormals->points[0]<<endl;
            // showNormals(sCloud, sNormals);
            // showNormals(tCloud, tNormals);
        }

        //---Extracting keypoints---
        PointCloud<Point_t>::Ptr sKeypoints (new PointCloud<Point_t>);
        PointCloud<Point_t>::Ptr tKeypoints (new PointCloud<Point_t>);

        if(DEBUG)
            cout<<"Detecting keypoints ...";

        detectIssKeypoints(sCloud, sKeypoints);
        detectIssKeypoints(tCloud, tKeypoints);
        // sKeypoints = sCloud;
        // tKeypoints = tCloud;

        if(DEBUG)
        {
            cout<<"OK"<<endl;
            cout<<"Amount of source keypoints: "<<sKeypoints->points.size()<<endl;
            cout<<"Amount of target keypoints: "<<tKeypoints->points.size()<<endl;
            // showKeypoints(sCloud, sKeypoints);
            // showKeypoints(tCloud, tKeypoints);
        }

        //---Computing descriptors---
        
        //Objects for storing descriptors
        PointCloud<Descriptor_t>::Ptr sDescriptors (new PointCloud<Descriptor_t>());
        PointCloud<Descriptor_t>::Ptr tDescriptors (new PointCloud<Descriptor_t>());
        if(DEBUG)
            cout<<"Computing FPFH descriptors ...";
        int k = 5; //descriptor's coordinates depend on k
        computeFpfhDescriptors(sCloud, sNormals, sKeypoints, sDescriptors, k);
        computeFpfhDescriptors(tCloud, tNormals, tKeypoints, tDescriptors, k);
        if(DEBUG)
        {
            cout<<"OK"<<endl;
            cout<<"First source FPFH descriptor:"<<sDescriptors->points[0]<<endl;
            cout<<"First target FPFH descriptor:"<<tDescriptors->points[0]<<endl;
        }
        

        //---Initial alignment---
        Eigen::Matrix4f transformation = alignWithSacIa(sKeypoints, tKeypoints, sDescriptors, tDescriptors);
        showAlignmentResult(sCloud, tCloud, transformation);
        // showAlignmentResult(sKeypoints, tKeypoints, transformation);
        

        // alignWithSacPrerejective(sCloud, tCloud, sKeypoints, tKeypoints, sDescriptors, tDescriptors);
        
        
        // int n = 125, 
        // k_ia = 5,
        // N = 1000;
        // float d = 0.5f;
        // // d = 2.5f * leafSize;
        // double threshold = 0.005;
        // Eigen::Matrix4f transformation = alignWithSacIa(sKeypoints, tKeypoints, sDescriptors, tDescriptors, n, d, k_ia, N, threshold);
        // PointCloud<Point_t>::Ptr transformed (new PointCloud<Point_t>());
        // transformPointCloud(*sCloud, *transformed, transformation);
        // showPointClouds(tCloud, transformed);
    }
}

void checkIss()
{
    bool fullCloud = false;
    string cloudPath;

    if(fullCloud)
    {
        cloudPath = "../data/full_ascii/view1.ply";
        // cloudPath = "../data/full_ascii/view2cc.ply";
        // cloudPath = "../data/full_ascii/view3.ply";
        // cloudPath = "../data/full_ascii/view4.ply";
    }
    else
    {
        cloudPath = "../data/cropped/view1ca.ply";
        // cloudPath = "../data/cropped/view2сa.ply";
    }


    //---Parameters for functions---
    float leafSize = 4.0;//downsampling parameters
    float radius = 2 * leafSize; //normal estimation parameter
    // float radius = 5.0f;

    //Object for storing point clouds
    PointCloud<Point_t>::Ptr cloud(new PointCloud<Point_t>);
    
    bool isLoaded = loadPlyCloud(cloudPath, cloud);

    if(isLoaded)
    {
        
        //---Downsampling---
        cloud = downsample(cloud, leafSize);

        if(fullCloud)
        {
            //---Remove plane---
            cloud = removePlane(cloud);
        }

        //---Filtering cloud---
        cloud = removeOutliersStatistically(cloud, 50, 1.0);

        //---Transition to the mass center point---
        transitToMassCenter(cloud);

        if(DEBUG)
            cout<<"Amount of points: "<<cloud->points.size()<<endl;


        //---Estimating normals---
        if(DEBUG)
            cout<<"Estimating normals ...";

        PointCloud<Normal>::Ptr normals = estimateSurfaceNormals(cloud, radius);

        if(DEBUG)
        {
            cout<<"OK"<<endl;
            cout<<"Amount of normals: "<<normals->points.size()<<endl;
            cout<<"First normal:"<<normals->points[0]<<endl;
            // showNormals(sCloud, normals);
        }

        //---Extracting keypoints---
        PointCloud<Point_t>::Ptr keypoints (new PointCloud<Point_t>);

        if(DEBUG)
            cout<<"Detecting keypoints ...";



        int minNeighbors = 5;
        float threshold21 = 0.975;
        float threshold32 = 0.975;
        int numberOfThreads = 4;
        

        detectIssKeypoints(cloud, keypoints, minNeighbors, threshold21, threshold32, numberOfThreads);

        if(DEBUG)
        {
            cout<<"OK"<<endl;
            cout<<"Amount of source keypoints: "<<keypoints->points.size()<<endl;
            showKeypoints(cloud, keypoints);
        }
    }

}


/**
 * Namespace that contains functions for testing
 */
namespace testing
{


    
    void testFiltering(string cloudPath)
    {

        PointCloud<Point_t>::Ptr cloud(new PointCloud<Point_t>);
        bool isLoaded = loadPlyCloud(cloudPath, cloud);
        if(isLoaded)
        {
            //---Downsampling---
            PointCloud<Point_t>::Ptr dcloud = downsample(cloud, 4.0);
            //---Remove plane---
            PointCloud<Point_t>::Ptr pcloud = removePlane(dcloud);
            //showInTwoWindows(sCloud, tCloud);

            //---Filtering cloud---
            
            PointCloud<Point_t>::Ptr fcloud;
            fcloud = removeOutliersStatistically(pcloud, 50, 1.0);

            //---Clusterization---

            vector<PointCloud<Point_t>::Ptr> clusteredClouds = getEuclideanClusters(fcloud, 15, 100);

            PointCloud<Point_t>::Ptr concatenatedCloud = concatenateClouds(clusteredClouds);

            showPointCloudSingleColor(concatenatedCloud);

            /*Color ar[] = {LightSeaGreen, LimeGreen, Purple, DarkOrange, Gold};
            std::vector<Color> colors(ar, ar + 5);
            cout << "Number of clusters: "<< clusteredClouds.size() << endl;
            showPointClouds(clusteredClouds, colors);*/


        }
    }


    void testRegistration(string sPath, string cPath)
	{
		//СОВМЕЩЕНИЕ ХОРОШО РАБОТАЕТ ДЛЯ ОБЛАКА И ЕГО МОДИФИКАЦИИ

	    float leafSize = 4.0;//downsampling parameters
	    float radius = 2 * leafSize; //normal estimation parameter
	    // float radius = 5.0f;

	    PointCloud<Point_t>::Ptr sCloud(new PointCloud<Point_t>);
	    PointCloud<Point_t>::Ptr cCloud(new PointCloud<Point_t>);

	    if(loadPlyCloud(sPath, sCloud) && loadPlyCloud(cPath, cCloud))
	    {
	        
	        //---Downsampling---
	        sCloud = downsample(sCloud, leafSize);
	        cCloud = downsample(cCloud, leafSize);

	        // cout<<"Before: "<<sCloud->points.size()<<endl;
	        // cout<<"After: "<<cCloud->points.size()<<endl;

	        //---Remove plane---
	        sCloud = removePlane(sCloud);
	        cCloud = removePlane(cCloud);

	        //---Filtering cloud---
	        sCloud = removeOutliersStatistically(sCloud, 50, 1.0);
	        cCloud = removeOutliersStatistically(cCloud, 50, 1.0);

	        //---Transition to the mass center point---
	        transitToMassCenter(sCloud);
	        transitToMassCenter(cCloud);

	        //---Reducing small clusters
	        vector<PointCloud<Point_t>::Ptr> sClusters = getEuclideanClusters(sCloud, 10, 200);
	        sCloud = concatenateClouds(sClusters);

	        vector<PointCloud<Point_t>::Ptr> cClusters = getEuclideanClusters(cCloud, 10, 200);
	        cCloud = concatenateClouds(cClusters);

	        // showInTwoWindows(sCloud, cCloud);


	        //---Estimating normals---
	        PointCloud<Normal>::Ptr sNormals = estimateSurfaceNormals(sCloud, radius);
	        PointCloud<Normal>::Ptr cNormals = estimateSurfaceNormals(cCloud, radius);
	        // showNormals(sCloud, sNormals);

	        //---Extracting keypoints---
	        PointCloud<Point_t>::Ptr sKeypoints (new PointCloud<Point_t>);
	        PointCloud<Point_t>::Ptr cKeypoints (new PointCloud<Point_t>);

	        //-->>>>--WORKS GOOD OR BAD???
	        
	        detectIssKeypoints(sCloud, sKeypoints, 10);

	        // PointCloud<Point_t>::Ptr sKeypointsSift (new PointCloud<Point_t>);
	        // detectSiftKeypoints(sCloud, sNormals, sKeypointsSift, 2);

	        
	        // showKeypoints(sCloud, sKeypointsSift );

	        detectIssKeypoints(cCloud, cKeypoints, 10);
	        // showKeypoints(sCloud, sKeypoints);
	        // showKeypoints(cCloud, cKeypoints);

	        //---Computing descriptors---
	        
	        //Objects for storing descriptors
	        PointCloud<Descriptor_t>::Ptr sDescriptors (new PointCloud<Descriptor_t>());
	        PointCloud<Descriptor_t>::Ptr cDescriptors (new PointCloud<Descriptor_t>());
	        int k = 5; //descriptor's coordinates depend on k
	        computeFpfhDescriptors(sCloud, sNormals, sKeypoints, sDescriptors, k, 10);
	        computeFpfhDescriptors(cCloud, cNormals, cKeypoints, cDescriptors, k, 10);
 			// cout<<"First source FPFH descriptor:"<<sDescriptors->points[0]<<endl;

	        Eigen::Matrix4f transformation = alignWithSacIa(sKeypoints, cKeypoints, sDescriptors, cDescriptors, 100, 5);
        	showAlignmentResult(sCloud, cCloud, transformation);
	     
	        // showInTwoWindows(sCloud, cCloud);

	    }
	}
}


/**
 * 	Contains all functions, that are used to create user interface
 */
namespace UI
{

	string choosePathToPointCloud(int index)
	{
		string path;// = "../data/full_ascii/view1.ply";
        switch(index)
        {
            case 1:
                path = "../data/full_ascii/view1.ply";
                break;
            case 2:
                path = "../data/full_ascii/view2cc.ply";
                break;
            default:
            	path = "";
            	break;
        }
        return path;
	}

	void launchMenu()
	{
		// cout << "Choose point cloud to load (1..4):/n"<<endl;
		// int pc = 0;
		// cin >> pc;
		string path1 = choosePathToPointCloud(1);
		string path2 = choosePathToPointCloud(2);
		testing::testRegistration(path1, path2);
		//testing::testFiltering(path);
	}
}



#endif