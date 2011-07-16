#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <body_msgs/Skeletons.h>
#include <sensor_msgs/PointCloud.h>
#include <body_msgs/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
//#include <pcl_tools/segfast.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace std;

struct HandSaver
{
 private:
  ros::NodeHandle n_;
  ros::Subscriber cloudsub_, skelsub_;
  sensor_msgs::PointCloud2 pcloudmsg;
  body_msgs::Skeletons skelmsg;
  //stringstream filename;
  std::string name;
  int count ;
  int lastskelseq, lastcloudseq;
  
public:
  
  HandSaver(std::string name_):name("Caputure")
    {
      cloudsub_ = n_.subscribe("hand1_fullcloud", 1, &HandSaver::cloudcb, this);      
      skelsub_ = n_.subscribe("/skeletons", 1, &HandSaver::skelcb, this);
      count = 0;
      lastskelseq = 0;
      lastcloudseq = 0;
      skelmsg.header.seq = 0;
      pcloudmsg.header.seq = 0;
      string cmd;
      //cmd = "mkdir " + name;
      //system(cmd.c_str());
     
    }
  

  // \brief This function tries to sync the skeleton and point cloud messages 
  void messageSync()
  {
    if ( skelmsg.header.seq == lastskelseq || pcloudmsg.header.seq == lastcloudseq )
      return;

    double tdiff = (skelmsg.header.stamp - pcloudmsg.header.stamp).toSec();
    
    //if (fabs(tdiff) < .15){
      lastskelseq = skelmsg.header.seq;
      lastcloudseq = pcloudmsg.header.seq;
      ProcessData(skelmsg, pcloudmsg);
      //}


  }
  void saveSkeletonsFileASCII( const char * fname, body_msgs::Skeleton skels)
  {
    ofstream out(fname);
    out << "left_hand " << skels.left_hand.position.x 
	<< " " << skels.left_hand.position.y
        << " " << skels.left_hand.position.z
        << " " << skels.left_hand.confidence 
        << std::endl;
    
    out << "right_hand " << skels.right_hand.position.x 
	<< " " << skels.right_hand.position.y
        << " " << skels.right_hand.position.z
        << " " << skels.right_hand.confidence 
        << std::endl;
    
    out << "left_elbow " << skels.left_elbow.position.x 
	<< " " << skels.left_elbow.position.y
        << " " << skels.left_elbow.position.z
        << " " << skels.left_elbow.confidence 
        << std::endl;
    
    out << "right_elbow " << skels.right_elbow.position.x 
	<< " " << skels.right_elbow.position.y
        << " " << skels.right_elbow.position.z
        << " " << skels.right_elbow.confidence;

    out.close();
    

  }
  void ProcessData( body_msgs::Skeletons skels, sensor_msgs::PointCloud2 cloud)
  {
    if (skels.skeletons.size() == 0)
      return;

    std::stringstream filename;
    pcl::PointCloud<pcl::PointXYZ> handcloud;
    pcl::fromROSMsg( cloud, handcloud);
    filename.str("");
    filename << name << "/" << setfill('0') << setw(4) << count << ".pcd";
    pcl::io::savePCDFileASCII( filename.str().c_str(), handcloud);

    filename.str("");
    filename << name << "/" << setfill('0') << setw(4) << count << ".txt";
    saveSkeletonsFileASCII( filename.str().c_str(), skels.skeletons[0]);
    std::cout << count << " "  <<  lastskelseq << " " << lastcloudseq << endl;
    count++;
   

  }
  void cloudcb( const sensor_msgs::PointCloud2ConstPtr &scan )
  {
    /*if ( scan)
      {  pcl::PointCloud<pcl::PointXYZ> handcloud;
    std::stringstream filename;
    pcloudmsg = *scan;
    pcl::fromROSMsg( pcloudmsg, handcloud);
    filename.str("");
    filename << "data/" << name << count << ".pcd";
    pcl::io::savePCDFileASCII( filename.str().c_str(), handcloud );
    count++;
      }

    */

    pcloudmsg = *scan;
    messageSync();
  }
  
  void skelcb ( const body_msgs::SkeletonsConstPtr &skels)
  {
    skelmsg = *skels;

    messageSync();

  }

};

int main( int argc, char ** argv )
{
  ros::init( argc, argv, "hand_saver");
  ros::NodeHandle n;
  std::string name;
  std::cout << "Please input the filename: ";
  std::cin >> name;
  HandSaver  saver(name);
  ros::spin();

  return 0;
}
