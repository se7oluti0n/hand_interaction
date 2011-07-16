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

int main(int argc, char **argv)
{
  cout << "The name used to start the program: " << argv[ 0 ]
       << "\nArguments are:\n";
  for (int n = 1; n < argc; n++)
    cout << setw( 2 ) << n << ": " << argv[ n ] << '\n';

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 1; i < argc; i++ )
    {
      pcl::PointCloud<pcl::PointXYZ> tmp;
      if ( pcl::io::loadPCDFile( argv[i], tmp) == -1 )
	{
	  cout << "Cant load file " << argv[i] << endl;
	  continue;
	}
      cloud+= tmp; 	  
    }

  string name;
  cout << "Input file name to save: ";
  cin >> name;
  
  pcl::io::savePCDFileASCII( name.c_str(), cloud);
}
