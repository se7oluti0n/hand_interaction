#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include <body_msgs/Skeletons.h>
#include <sensor_msgs/PointCloud.h>
#include <body_msgs/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
//#include <pcl_tools/segfast.hpp>
#include "pcl/common/transform.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
 main (int argc, char** argv)
{
  string name;
  cout << "Input filename : " ;
  cin >> name;
  float radius;
  cout << "Search radius: " ;
  cin >> radius;
  
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  sensor_msgs::PointCloud2 cloud_blob;
  // Load bun0.pcd -- should be available with the PCL archive in test 
  pcl::io::loadPCDFile (name.c_str(), cloud_blob);
  pcl::fromROSMsg (cloud_blob, *cloud);

  // Create a KD-Tree
  pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  // Output has the same type as the input one, it will be only smoothed
  pcl::PointCloud<pcl::PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
  mls.setOutputNormals (mls_normals);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (radius);

  // Reconstruct
  mls.reconstruct (mls_points);

  // Concatenate fields for saving
  pcl::PointCloud<pcl::PointNormal> mls_cloud;
  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);


  cout << "Output filename : " ;
  cin >> name;

  // Save output
  pcl::io::savePCDFile (name.c_str(), mls_cloud);
  return 0;

}
