#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
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

boost::mt19937 gen;
struct arms
{
  body_msgs::SkeletonJoint left_hand;
  body_msgs::SkeletonJoint left_elbow;
  body_msgs::SkeletonJoint right_hand;
  body_msgs::SkeletonJoint right_elbow;
};

void readJointFile(char *name, arms &a)
{
  ifstream in(name);
  string jointname;
 
  cout << name << " " ;
  in >> jointname;
  in >> a.left_hand.position.x;
  cout << a.left_hand.position.x;
  in >> a.left_hand.position.y;
  in >> a.left_hand.position.z;
  in >> a.left_hand.confidence;
  
  in >> jointname;
  in >> a.right_hand.position.x;
  in >> a.right_hand.position.y;
  in >> a.right_hand.position.z;
  in >> a.right_hand.confidence;
  
  in >> jointname;
  in >> a.left_elbow.position.x;
  in >> a.left_elbow.position.y;
  in >> a.left_elbow.position.z;
  in >> a.left_elbow.confidence;
  
  in >> jointname;
  in >> a.right_elbow.position.x;
  in >> a.right_elbow.position.y;
  in >> a.right_elbow.position.z;
  in >> a.right_elbow.confidence;
  
}
/* \brief : Resampling point from given center point and error
 *
 *
 *
 *
 */
pcl::PointXYZ draw_sample ( pcl::PointXYZ center, float error )
{
 
  float min = center.z - error / 2;
  float max = center.z + error / 2;

  //boost::random::mt19937 rng;         // produces randomness out of thin air
                                    // see pseudo-random number generators
  boost::uniform_real<> u(min,max);
                                    // distribution that maps to 1..6
                                    // see random number distributions
  float x = u(gen);                   // simulate rolling a die
  
  return pcl::PointXYZ( center.x, center.y, x );

}
/* \brief 
 * 
 *
 *
 *
 */ 
void resample( pcl::PointCloud<pcl::PointXYZ> & cloudin, pcl::PointCloud<pcl::PointXYZ> & cloudout, arms &armin )
{
  float baseline = 0.07219;
  float disparity_error = 0.17;
  float focal_length = 580;
  float z = armin.right_hand.position.z;
  int sample_point = 6000 / cloudin.points.size() + 1;
  
  float error = disparity_error * z * z / focal_length / baseline;
  
  for (int i = 0; i < cloudin.points.size(); i++ )
  {
    for ( int j = 0; j < sample_point; j++ )
      cloudout.push_back( draw_sample ( cloudin.points[i], error));
  }



}
/* \brief : Transform a Pointcloud from Kinect coordinator to Hand Coordinator
 *
 *
 *
 *
 */
void getTransfromation(pcl::PointCloud<pcl::PointXYZ> &cloudin, arms &armin, Eigen::Affine3f &transformation, float & arm_length)
{
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  // Eigen::Matrix3f cov;
  Eigen::Vector4f centroid, dirdection,armvector;
  Eigen::Vector3f z_axis, y_axis, x_axis, origin;

  Eigen::Vector3f right_arm, yvector;

  yvector[0]  = 0;
  yvector[1]  = 1;
  yvector[2]  = 0;

  right_arm[0] = armin.right_hand.position.x - armin.right_elbow.position.x;
  right_arm[1] = armin.right_hand.position.y - armin.right_elbow.position.y;
  right_arm[2] = armin.right_hand.position.z - armin.right_elbow.position.z;
  
  arm_length = right_arm.norm();
  /*
  zvector[0] = 0;
  zvector[1] = 0;
  zvector[2] = 1;

  // double dot = right_arm.dot(zvector);
  //cout << "x: " << right_arm[0] << " y : " << right_arm[1] << " dot: " << dot;
  // cross = zvector.cross(right_arm);
  
  double tann = right_arm[0] / right_arm[2];
  
  right_arm[1] = armin.right_hand.position.y - armin.right_elbow.position.y;  
  */

   pcl::compute3DCentroid (cloudin, centroid);
   // pcl::computeCovarianceMatrixNormalized(cloudin,centroid,cov);
   // pcl::eigen33 (cov, eigen_vectors, eigen_values);

   
   z_axis[0] = right_arm[0];//eigen_vectors( 0, 2);
   z_axis[1] = right_arm[1];//eigen_vectors( 1, 2);
   z_axis[2] = right_arm[2];//eigen_vectors( 2, 2);
   

   x_axis = yvector.cross(right_arm);
   double cos = right_arm.dot(z_axis) / sqrt( right_arm.norm() * z_axis.norm() );
   if ( cos < 0 ) x_axis = - x_axis;
    
   y_axis = z_axis.cross(x_axis);
   x_axis.normalize();
   y_axis.normalize();
   z_axis.normalize();
   
   
   /*cout << " tan: "<< tann ;
    if ( tann <= -3.7 || tann >= 3.7 ) 
      {
	y_axis[0] = eigen_vectors( 0, 1);
	y_axis[1] = eigen_vectors( 1, 1);
	y_axis[2] = eigen_vectors( 2, 1);
      }
    else
      {
	y_axis[0] = eigen_vectors( 0, 0);
	y_axis[1] = eigen_vectors( 1, 0);
	y_axis[2] = eigen_vectors( 2, 0);
      }
  
    cross =  right_arm.cross(y_axis);
    double sin = cross.norm() / right_arm.norm() / y_axis.norm();


    if ( sin > 0 ) y_axis = -y_axis;
    cout << " sin: " << sin << " cos: " << cos << endl;
    /*   x_axis[0] = eigen_vectors( 0, 1);
    x_axis[1] = eigen_vectors( 1, 1);
    x_axis[2] = eigen_vectors( 2, 1);

    

    /* z_axis[0] = 0;//eigen_vectors( 0, 2);
    z_axis[1] = 0;//eigen_vectors( 1, 2);
    z_axis[2] = 1;//eigen_vectors( 2, 2);
    
    y_axis[0] = 0;//eigen_vectors( 0, 0);
    y_axis[1] = 1;//eigen_vectors( 1, 0);
    y_axis[2] = 0;//eigen_vectors( 2, 0);
    */

   /*  y_axis[0] = eigen_vectors( 0, 0);
    y_axis[1] = eigen_vectors( 1, 0);
    y_axis[2] = eigen_vectors( 2, 0);

   */
    origin [ 0 ] = armin.right_hand.position.x;
    origin [ 1 ] = armin.right_hand.position.y;
    origin [ 2 ] = armin.right_hand.position.z;
   

   pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_axis, z_axis, origin, transformation);
}
int main(int argc, char ** argv)
{
  //ros::init(argc, argv, "test");
  int classs;
  cout << "Input class label( 1 / 0 ): " ;
  cin >> classs;
  
  
  system("ls *pcd > PCDfileList");
  system("ls *txt > SkelFileList");
  system("mkdir converted");
  system("mkdir histogram");
  
   
  std::stringstream filename;
  int count = 0;
  ifstream pcdin("PCDfileList");
  ifstream skelin("SkelFileList");
  ofstream out("trainingdata.txt");
  while ( ! pcdin.eof() )
  {
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud2, output;
    Eigen::Affine3f transformation;
    float arm_length;
 
    cout << "Start processing file " << count << endl;
    char name[256];
   pcdin.getline( name, 256 );
    
   if ( pcl::io::loadPCDFile( name, cloud ) == -1 )
     {
       // PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
       break;
     }
   
   skelin.getline( name, 256 );

   arms aa;
   readJointFile(name, aa);
   cout << count << " : " ;
   resample( cloud, cloud2, aa);
   getTransfromation( cloud2, aa, transformation, arm_length);
   pcl::getTransformedPointCloud (cloud2, transformation, output);

   pcl::PointXYZ min_pt, max_pt;

   pcl::getMinMax3D( output, min_pt, max_pt);


   Eigen::Vector3f origin;
   origin[0] = aa.right_hand.position.x;
   origin[1] = aa.right_hand.position.y;
   
   
    //pcl::demeanPointCloud(cloud, centroid, output);
    filename.str("");
    
    filename << "converted/" << setfill('0') << setw(4) << count << ".pcd";
    pcl::io::savePCDFileASCII( filename.str().c_str(), output);
    cout << "Wrote : "  << filename.str() << endl;
    cout << " Max : " << max_pt.z << " " << max_pt.y << " " << max_pt.x << endl;
    cout << " Min : " << min_pt.z << " " << min_pt.y << " " << min_pt.x << endl;
    
    
    
    /* float r1 = max_pt.y * max_pt.y + max_pt.x * max_pt.x  ;
    float r2 = min_pt.y * min_pt.y + min_pt.x * min_pt.x;
    float r3 = max_pt.y * max_pt.y + min_pt.x * min_pt.x;
    float r4 = min_pt.y * min_pt.y + max_pt.x * max_pt.x;

    
    
    float r = r1 > r2?r1:r2;
    r = r > r3?r:r3;
    r = r > r4?r:r4;
    */
    float r = 0.8 * arm_length;
    
    float offset_r = r /4.999;
    origin[2] = min_pt.z;
    
    float offset_z = 0.8 * arm_length / 5.999;
    const double PI = 3.14159266;
    float offset_a = PI / 3.999;

    float histogram[240];
   
    for ( int i = 0; i < 240; i++ )
      
      {
	histogram[i] = 0.0;
	//	cout << histogram[i];
      }
     for ( int i = 0; i < output.points.size(); i++ )
      {
	int zz = floor (( output.points[i].z - origin[2] ) / offset_z);
	zz = ( zz > 5?5:zz);
	int pp = floor (( output.points[i].y * output.points[i].y  + output.points[i].x * output.points[i].x ) / offset_r );
	pp = ( pp > 4?4:pp);
	int aa = floor(( PI + atan2(output.points[i].y, output.points[i].x)) / offset_a );

        int index = zz * 40 + pp * 8 + aa % 8;
	if ( index < 0 || index > 239 ) 
	  {
	    cout << "Out of range!!" << " " << zz << " " << pp << " " << aa << endl;
	    exit(1);
	  }
	histogram[index] += 1.0;
      }
     cout << "Complete histogram" << endl;
     filename.str("");
     filename << "histogram/" << setfill('0') << setw(4) << count << ".data";

    
     for ( int i = 0; i < 240; i++ )
    {
      histogram[i] /= (float) output.points.size();
     
     
      // tcout << histogram[i] << endl;
    }
    
     cout << "Complete Normalized" << endl;
     
     ofstream hisout(filename.str().c_str());
     for ( int i = 0; i < 240; i++ )
       {
	 hisout << i << " " << histogram[i] << endl;
	 	 
       }
     
     hisout.close();
     cout << "Complete write to Histogram folder" << endl;
    

       out << classs;
   
       for ( int i = 0; i < 240; i++ )
	 {
	   out << " " << i+1 << ":" <<  histogram[i];
	 }
    
       out << endl;
    

     cout << "Complete write to training file" << endl;
     // getTransFromUnitVectorsZY (z_axis, y_axis, transformation);
   
   
    count++;
     cout << "Complete write to training file" << endl;
    }

    out.close();
  return 0;
}
