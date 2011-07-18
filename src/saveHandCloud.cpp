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

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
struct HandSaver
{
 private:
  ros::NodeHandle n_;
  ros::Subscriber cloudsub_, skelsub_, imgsub_;
  sensor_msgs::PointCloud2 pcloudmsg;
  body_msgs::Skeletons skelmsg;
  sensor_msgs::ImageConstPtr imgmsg;
  //stringstream filename;
  std::string name;
  int count ;
  int lastskelseq, lastcloudseq;
  
public:
  
  HandSaver(std::string name_):name(name_)
    {
      cloudsub_ = n_.subscribe("hand1_fullcloud", 1, &HandSaver::cloudcb, this);      
      skelsub_ = n_.subscribe("/skeletons", 1, &HandSaver::skelcb, this);
      imgsub_ = n_.subscribe("/camera/rgb/image_color",1, &HandSaver::imgcb, this);
      count = 0;
      lastskelseq = 0;
      lastcloudseq = 0;
      skelmsg.header.seq = 0;
      pcloudmsg.header.seq = 0;
      string cmd;
      cmd = "mkdir " + name;
      system(cmd.c_str());
     
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


  /** \fn void saveSkeletonFileASCII ( const char * fname, body_msgs::Skeleton skels
   *  \brief Save a skeleton data \a skels to a ASCII file named \a fname
   *  \param fname File name
   *  \param skels Skeleton data struct
   *
   */
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

    filename.str("");
    filename << name << "/" << setfill('0') << setw(4) << count << ".jpg";
    cv_bridge::CvImageConstPtr imgptr;

    imgptr = cv_bridge::toCvShare( imgmsg, enc::BGR8);
    cv::imwrite(filename.str().c_str(), imgptr->image);
    //cv::Mat img = imgptr->image.clone();
    // imageHandDetect(img, );

    /* float constant = 1.90476e-03;
    float centerX = 319.5;
    float centerY = 239.5;
    float radius = 0.1;
    double PI = 3.14159265;
    
    int u,v,r, u1, v1;
    u = (int ) ( skels.skeletons[0].right_hand.position.x / constant / skels.skeletons[0].right_hand.position.z + centerX ); 
    v = (int ) ( skels.skeletons[0].right_hand.position.y / constant / skels.skeletons[0].right_hand.position.z + centerY ); 

    u1 = (int ) ( skels.skeletons[0].right_elbow.position.x / constant / skels.skeletons[0].right_elbow.position.z + centerX ); 
    v1 = (int ) ( skels.skeletons[0].right_elbow.position.y / constant / skels.skeletons[0].right_elbow.position.z + centerY ); 
    
    double angle;
    angle = atan2(skels.skeletons[0].right_hand.position.y- skels.skeletons[0].right_elbow.position.y , skels.skeletons[0].right_hand.position.x - skels.skeletons[0].right_elbow.position.x);
    
    angle = 180 * angle / PI;

    cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2f(centerX,centerY), angle, 1.0);
    cv::Mat iRot_mat, xx, xx1;
    cv::invertAffineTransform(rot_mat, iRot_mat);
    xx.create(3,1,CV_64FC1);
    //xx1.create(2,1,CV_32FC1);
    r = (int ) ( radius / constant / skels.skeletons[0].right_hand.position.z); 
    cout << " constant : " << constant << " " << centerX << " " << centerY << endl;
    cout << " x y z " << skels.skeletons[0].right_hand.position.x << " " << skels.skeletons[0].right_hand.position.y << " "<< skels.skeletons[0].right_hand.position.z << endl;
    cout << " UV : " << u << " " << v << endl;
    
    cv::line( img, cv::Point(u1,v1), cv::Point(u,v), cv::Scalar(0,0,255), 2);
    
    cv::Mat dst;       
    cv::warpAffine(img, dst, rot_mat,cv::Size(640,480));
    xx.at<float>(0,0) = (float)u ;
    xx.at<float>(1,0) = (float)v ;
    xx.at<float>(2,0) = 1;

    xx1 = rot_mat*xx;
    cout << "XX: "<< xx.at<float>(0,0) << " " << xx.at<float>(1,0) << endl;
    filename.str("");
    filename << angle;
    
    cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,255,0), 2);
    cv::circle( dst, cv::Point((int)( xx1.at<float>(0,0) + centerX),(int) ( xx1.at<float>(1,0)) + centerY) , r, cv::Scalar(0,0,255), 2);
    cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));*/
   
    /*cv::imshow("Hand Detect", img);
    cv::imshow("Rotation", dst);
    cv::waitKey(20);
    std::cout << count << " "  <<  lastskelseq << " " << lastcloudseq << endl;*/
    count++;
   

  }
  /**************************************************************************
    Image Subscriber Callback
     


  /**************************************************************************/ 
  void imgcb( const sensor_msgs::ImageConstPtr & img)
  {
    imgmsg = img;
    messageSync();

  }
  /** \fn void cloudcb
   *  \brief hand point cloud subscriber callback
   *
   *
   */


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
