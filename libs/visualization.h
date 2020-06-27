 /* \file
 * \brief Header file with functions to visualize point clouds
 */


#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "typedefs.h"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <boost/lexical_cast.hpp>

/**
 * @brief Struct that represents color\
 */
typedef struct Color
{
	int red;   ///< red color component
	int green; ///< green color component
	int blue;  ///< blue color component
} Color;

//---User defined colors---
Color Red = {255, 0, 0};
Color Green = {0, 255, 0};
Color Blue = {0, 0, 255};
Color LightSeaGreen = {32, 178, 170};
Color LimeGreen = {50, 205, 50};
Color Purple = {128, 0, 128};
Color DarkOrange = {255, 140, 0};
Color Gold = {255, 215, 0};
Color FireBrick = {178, 34, 34};
//------

/**
 * @brief Covert int value to string
 * 
 * @param[in] value input number
 * 
 * @return string representation of value
 */
std::string IntToString(int value)
{
    return boost::lexical_cast<std::string>(value);
}

/**
 * @brief Visualize keypoints
 * @details Open point cloud in pcl_visualizer and show keypoints
 * 
 * @param[in] cloud input point cloud
 * @param[in] keypoints input keypoints
 */
void showKeypoints(pcl::PointCloud<Point_t>::Ptr cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, Color color  = Red,
 std::string title = "keypoints")
{
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (cloud, title);    // Add the points to the vizualizer

    viz.setWindowName("Keypoints");
    viz.addText("KeyPoints: " + IntToString(keypoints->size()), 0, 0);

    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keypoints->size (); ++i)
    {
        pcl::PointWithScale & p = keypoints->points[i]; // Get the point data
        // float r = 5.0f;
        float r = 2 * p.scale;  // Pick the radius of the sphere
        // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
        //   radius of 2*p.scale is a good illustration of the extent of the keypoint

        std::string textId = "keypoint" + IntToString(i); // Generate a unique string for each sphere
        viz.addSphere (p, r, color.red / 255.0, color.green / 255.0, color.blue / 255.0, textId); // Add a sphere at the keypoint
    }
    viz.spin ();    // Give control over to the visualizer
}

/**
 * @brief Visualize keypoints
 * @details Open point cloud in pcl_visualizer and show keypoints
 * 
 * @param[in] cloud input point cloud
 * @param[in] keypoints input keypoints
 */
void showKeypoints(pcl::PointCloud<Point_t>::Ptr cloud, pcl::PointCloud<Point_t>::Ptr keypoints, Color color  = Red, 
    std::string title = "keypoints")
{
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (cloud, title);    // Add the points to the vizualizer

    viz.setWindowName("Keypoints");
    viz.addText("KeyPoints: " + IntToString(keypoints->size()), 100, 0);

    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keypoints->size (); ++i)
    {
        Point_t & p = keypoints->points[i]; // Get the point data
        float r = 5.0f;
        // float r = 2 * p.scale;  // Pick the radius of the sphere
        // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
        //   radius of 2*p.scale is a good illustration of the extent of the keypoint

        std::string textId = "keypoint" + IntToString(i); // Generate a unique string for each sphere
        // viz.addSphere (p, r, color.red, color.green, color.blue, title); // Add a sphere at the keypoint
        viz.addSphere (p, r, color.red / 255.0, color.green / 255.0, color.blue / 255.0, textId);
    }
    viz.spin ();    // Give control over to the visualizer
}

/**
 * @brief Show point cloud in CloudViewer
 * 
 * @param[in] cloud input point cloud
 */
void showPointCloud(pcl::PointCloud<Point_t>::Ptr &cloud, std::string title = "cloud")
{
	pcl::visualization::CloudViewer viewer(title);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
        // Do nothing but wait.
    }
}

/**
 * @brief Show point cloud painted in a single color
 * 
 * @param[in] cloud input point cloud
 * @param[in] color paint color, default is gold
 */
void showPointCloudSingleColor(pcl::PointCloud<Point_t>::Ptr &cloud, 
    Color color = Gold, std::string title = "cloud")
{
    pcl::visualization::PCLVisualizer viewer(title);
    pcl::visualization::PointCloudColorHandlerCustom<Point_t> colorHandler(cloud, color.red, color.green, color.blue);
    viewer.addPointCloud(cloud, colorHandler, "original");
    viewer.addCoordinateSystem(1000.0, "coordinate system", 0); // Add 3D colored axes to help see the transformation.
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(); ///  \todo find out why do we need this string
    }
}

/**
 * @brief Show 2 point clouds painted in 2 single colors
 * @todo Generalize input arguements
 * 
 * @param[in] cloudA, cloudB input point clouds
 * @param[in] colorA, colorB input colors of point clouds respectively, default are gold and light sea green
 */
void showPointClouds(pcl::PointCloud<Point_t>::Ptr &cloudA, pcl::PointCloud<Point_t>::Ptr &cloudB, 
	Color colorA = Gold, Color colorB = LightSeaGreen)
{
	pcl::visualization::PCLVisualizer viewer("Clouds");
    pcl::visualization::PointCloudColorHandlerCustom<Point_t> colorHandlerA(cloudA, colorA.red, colorA.green, colorA.blue);
	viewer.addPointCloud(cloudA, colorHandlerA, "original");
	pcl::visualization::PointCloudColorHandlerCustom<Point_t> colorHandlerB(cloudB, colorB.red, colorB.green, colorB.blue);
	viewer.addPointCloud(cloudB, colorHandlerB, "transformed");
	viewer.addCoordinateSystem(1000.0, "coordinate system", 0);    // Add 3D colored axes to help see the transformation.
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

void showPointClouds(std::vector<pcl::PointCloud<Point_t>::Ptr> &clouds, std::vector<Color> colors)
{
    pcl::visualization::PCLVisualizer viewer("Clouds");

    for (int i = 0; i < clouds.size(); ++i)
    {
        pcl::visualization::PointCloudColorHandlerCustom<Point_t> colorHandler(clouds[i], colors[i].red, colors[i].green, colors[i].blue);
        viewer.addPointCloud(clouds[i], colorHandler, "cloud #" + IntToString(i));
    }
    viewer.addCoordinateSystem(1000.0, "coordinate system", 0);    // Add 3D colored axes to help see the transformation.
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

/**
 * @brief Show point cloud with normals
 * 
 * @param[in] cloud input point cloud
 * @param[in] normals input normals
 * @param[in] normalsLength length of the line to draw for each normal
 * @param[in] numbOfNormsToDisplay number of normals to display, default shows every second normal
 */
void showNormals(pcl::PointCloud<Point_t>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
 float normalsLength = 10.0f, int numbOfNormsToDisplay = 2)
{
    pcl::visualization::PCLVisualizer viewer("Normals");
    viewer.addPointCloud<Point_t>(cloud, "cloud");
    viewer.addPointCloudNormals<Point_t, pcl::Normal>(cloud, normals, numbOfNormsToDisplay, normalsLength, "normals");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

/**
 * @brief Create independent area of window
 * 
 * @param viewer PCLVisualizer object
 * @param vp reference to viewport
 * @param xMin, yMin upper left corner coordinates of an area
 * @param xMax, yMax right bottom corner coordinates of an area
 */
void createIndependentViewPort(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int &vp, 
    float xMin, float yMin, float xMax, float yMax)
{
    viewer->createViewPort (xMin, yMin, xMax, yMax, vp);
    //viewer->setBackgroundColor (bgColor.red, bgColor.green, bgColor.blue, *vp);
    viewer->createViewPortCamera(vp);
}

/*char* concatStrings(char* wordA, char* wordB)
{
    char* word = new char[strlen(wordA) + strlen(wordB) + 1];
    strcat(word, wordA);
    strcat(word, wordB);
    return word;
}*/

/**
 * @brief Add multiple point cloud to visualizer
 * 
 * @param[in] clouds input point clouds
 * @param[in] viewer PCLVisualizer object
 * @param[in] vp reference to viewport
 * @param[in] title general title of input clouds dataset
 */
void addMultipleClouds(std::vector<pcl::PointCloud<Point_t>::Ptr> &clouds,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int &vp, std::string title)
{
    for (int i = 0; i < clouds.size(); ++i)
    {
        std::string uniqueTitle = title + IntToString(i); 
        pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgb (clouds[i]);
        viewer->addPointCloud<Point_t> (clouds[i], rgb, uniqueTitle, vp);
    }
}

/**
 * @brief Add multiple point cloud to visualizer with own colors
 * 
 * @todo Think out how to get rid of repetition
 * 
 * @param[in] clouds input point clouds
 * @param[in] colors input colors of point clouds
 * @param[in] viewer PCLVisualizer object
 * @param[in] vp reference to viewport
 * @param[in] title general title of input clouds dataset
 */
void addMultipleClouds(std::vector<pcl::PointCloud<Point_t>::Ptr> &clouds, std::vector<Color> &colors,
    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int &vp, std::string title)
{
    for (int i = 0; i < clouds.size(); ++i)
    {
        std::string uniqueTitle = title + IntToString(i); 
        if(i < colors.size())
        {
            pcl::visualization::PointCloudColorHandlerCustom<Point_t> singleColor (clouds[i], colors[i].red, colors[i].green, colors[i].blue);
            viewer->addPointCloud<Point_t> (clouds[i], singleColor, uniqueTitle, vp);
        }
        else
        {
            pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgb (clouds[i]);
            viewer->addPointCloud<Point_t> (clouds[i], rgb, uniqueTitle, vp);
        }
        
    }
}

//Shows two point clouds in two independent areas of window
/*void showInTwoWindows(pcl::PointCloud<Point_t>::Ptr &cloudA, pcl::PointCloud<Point_t>::Ptr &cloudB)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();


    int v1(0);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    //viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->createViewPortCamera(v1);
    // createIndependentViewPort(viewer, &v1, 0.0, 0.0, 0.5, 1.0);
    //TODO:Change text here
    viewer->addText ("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgbA (cloudA);
    viewer->addPointCloud<Point_t> (cloudA, rgbA, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    //viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->createViewPortCamera(v2);
    //createIndependentViewPort(viewer, &v2, 0.5, 0.0, 1.0, 1.0);
    //TODO:Change text here
    viewer->addText ("Radius: 0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgbB (cloudB);
    viewer->addPointCloud<Point_t> (cloudB, rgbB, "sample cloud2", v2);
    // pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color (cloudB, 0, 255, 0);

    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}*/

/**
 * @brief Show two point clouds in two areas of window painted in own colors
 * 
 * @todo Change this function inside
 * 
 * @param[in] cloudA, cloudB input point clouds
 * @param[in] colorA, colorB input colors of point clouds respectively, default are gold and light sea green
 */
void showInTwoWindows(pcl::PointCloud<Point_t>::Ptr &cloudA, pcl::PointCloud<Point_t>::Ptr &cloudB,
    Color colorA = Gold, Color colorB = LightSeaGreen)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    //viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->createViewPortCamera(v1); 
    // createIndependentViewPort(viewer, &v1, 0.0, 0.0, 0.5, 1.0);
    //TODO:Change text here
    viewer->addText ("Not aligned", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<Point_t> rgbA (cloudA, colorA.red, colorA.green, colorA.blue);
    viewer->addPointCloud<Point_t> (cloudA, rgbA, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    //viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->createViewPortCamera(v2);
    //createIndependentViewPort(viewer, &v2, 0.5, 0.0, 1.0, 1.0);
    //TODO:Change text here
    viewer->addText ("Aligned", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<Point_t> rgbB (cloudB, colorB.red, colorB.green, colorB.blue);
    viewer->addPointCloud<Point_t> (cloudB, rgbB, "sample cloud2", v2);
    // pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color (cloudB, 0, 255, 0);

    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1000.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * @brief Create independent area of window add point cloud to it
 * @details [long description]
 * 
 * @param viewer [description]
 * @param text Text that will be shown in this part of window
 * @param color Color of point cloud
 * @param pos Array that respresents size of this area: xmin, ymin, xmax, ymax
 * @param id Id of created area
 */
void addViewToVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<Point_t>::Ptr &cloud, 
    std::string text, Color color, float pos[], int id)
{
    int v(0);
    viewer->createViewPort (pos[0], pos[1], pos[2], pos[3], v);
    //viewer->setBackgroundColor (0, 0, 0, v);
    viewer->createViewPortCamera(v); 
    viewer->addText (text, 10, 10, "v" + IntToString(id) + "text", v);
    pcl::visualization::PointCloudColorHandlerCustom<Point_t> rgb (cloud, color.red, color.green, color.blue);
    viewer->addPointCloud<Point_t> (cloud, rgb, "sample cloud" + IntToString(id), v);
}

/**
 * @brief Create two independent windows with point clouds and with its unique text
 * @details [long description]
 * 
 * @param d [description]
 * @param textA [description]
 * @param textB [description]
 * @param colorA [description]
 * @param colorB [description]
 */
void showInTwoWindows(pcl::PointCloud<Point_t>::Ptr &cloudA, pcl::PointCloud<Point_t>::Ptr &cloudB, std::string textA, std::string textB,
    Color colorA = Gold, Color colorB = LightSeaGreen)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    addViewToVisualizer(viewer, cloudA, textA, colorA, new float[4]{0.0, 0.0, 0.5, 1.0}, 1);
    addViewToVisualizer(viewer, cloudB, textB, colorB, new float[4]{0.5, 0.0, 1.0, 1.0}, 2);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1000.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


/**
 * @brief Show two sets of point clouds in two independent areas of window
 * 
 * @param[in] cloudsA, cloudsB input sets of point clouds
 */
void showInTwoWindows(std::vector<pcl::PointCloud<Point_t>::Ptr> &cloudsA, std::vector<pcl::PointCloud<Point_t>::Ptr> &cloudsB)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1 = 0;
    createIndependentViewPort(viewer, v1, 0.0, 0.0, 0.5, 1.0);
    //viewer->setBackgroundColor (0, 0, 0, v1);
    
    //TODO:Change text here
    viewer->addText ("Clouds A", 10, 10, "v1 text", v1);
    addMultipleClouds(cloudsA, viewer, v1, "title1");

    int v2 = 0;
    createIndependentViewPort(viewer, v2, 0.5, 0.0, 1.0, 1.0);
    viewer->setBackgroundColor (0.1, 0.1, 0.1, v2);
    
    //TODO:Change text here
    viewer->addText ("Clouds B", 10, 10, "v2 text", v2);
    addMultipleClouds(cloudsB, viewer, v2, "title2");

    //TODO: Think about it
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1000.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

/**
 * @brief Shows two sets of point clouds in two independent areas of window painted in single colors
 * 
 * @todo Set all parameters
 * 
 * @param[in] cloudsA, cloudsB input sets of point clouds
 * @param[in] colorsA, colorsB input colors of point clouds in two sets
 */
void showInTwoWindows(std::vector<pcl::PointCloud<Point_t>::Ptr> &cloudsA, std::vector<pcl::PointCloud<Point_t>::Ptr> &cloudsB, 
    std::vector<Color> &colorsA, std::vector<Color> &colorsB)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1 = 0;
    createIndependentViewPort(viewer, v1, 0.0, 0.0, 0.5, 1.0);
    //viewer->setBackgroundColor (0, 0, 0, v1);
    
    //TODO:Change text here
    viewer->addText ("Not aligned", 10, 10, "v1 text", v1);
    addMultipleClouds(cloudsA, colorsA, viewer, v1, "title1");

    int v2 = 0;
    createIndependentViewPort(viewer, v2, 0.5, 0.0, 1.0, 1.0);
    viewer->setBackgroundColor (0.1, 0.1, 0.1, v2);
    
    //TODO:Change text here
    viewer->addText ("Aligned", 10, 10, "v2 text", v2);
    addMultipleClouds(cloudsB, colorsB, viewer, v2, "title2");

    //TODO: Think about it
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1000.0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

//Works very slow!
void showIn4Windows(std::vector<pcl::PointCloud<Point_t>::Ptr> &clouds, std::string *texts, Color *colors)
{ 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    float positions[4][4] = {{0.0, 0.0, 0.5, 0.5}, {0.5, 0.0, 1, 0.5}, {0.0, 0.5, 0.5, 1.0}, {0.5, 0.5, 1.0, 1.0}};
    for (int i = 0; i < 4; ++i)
    {
        addViewToVisualizer(viewer, clouds[i], texts[i], colors[i], positions[i], i);
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    }
    viewer->addCoordinateSystem (1000.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (10);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
}

#endif