/* Detect Pointing area
 *
 *
 *
 *
 *
 *
 */


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <body_msgs/Skeletons.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <nnn/nnn.hpp>


#include <pcl/common/transform.h>
#include <pcl/common/norms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;



float gdist(pcl::PointXYZ pt, const Eigen::Vector4f &v){
   return sqrt((pt.x-v(0))*(pt.x-v(0))+(pt.y-v(1))*(pt.y-v(1))+(pt.z-v(2))*(pt.z-v(2))); //
}

void flipvec(const Eigen::Vector4f &palm, const Eigen::Vector4f &fcentroid,Eigen::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

template <typename Point1, typename Point2>
void PointConversion(Point1 pt1, Point2 &pt2){
   pt2.x=pt1.x;
   pt2.y=pt1.y;
   pt2.z=pt1.z;
}


geometry_msgs::Point32 eigenToMsgPoint32(const Eigen::Vector4f &v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(const Eigen::Vector4f &v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

pcl::PointXYZ eigenToPclPoint(const Eigen::Vector4f &v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}


geometry_msgs::Transform pointToTransform(geometry_msgs::Point p){
   geometry_msgs::Transform t;
   t.translation.x=p.x; t.translation.y=p.y; t.translation.z=p.z;
   return t;
}

pcl::PointXYZ pointToPclPoint(geometry_msgs::Point p){
   pcl::PointXYZ p1;
   p1.x=p.x; p1.y=p.y; p1.z=p.z;
   return p1;
}

//adds a set amount (scale) of a vector from pos A to pos B to point C
//this function is mostly here to do all the nasty conversions...
pcl::PointXYZ addVector(const pcl::PointXYZ &_C, geometry_msgs::Point A, geometry_msgs::Point B, double scale){
  pcl::PointXYZ C=_C;
  (C.x) += scale*(B.x-A.x);
  (C.y) += scale*(B.y-A.y);
  (C.z) += scale*(B.z-A.z);
  return C;
}


bool isJointGood(body_msgs::SkeletonJoint &joint){
   if(joint.confidence < 0.5)
      return false;
   else
      return true;
}

struct Pointing
{
private:
  ros::NodeHandle n_;
  ros::Subscriber cloudsub_, skelsub_, imgsub_;
  ros::Publisher detepub_[2];
  sensor_msgs::PointCloud2 pcloudmsg;
  body_msgs::Skeletons skelmsg;
  sensor_msgs::ImageConstPtr imgmsg;


  int count ;
  int lastskelseq, lastcloudseq;

public:
  Pointing()
  {
    cloudsub_ = n_.subscribe("/camera/rgb/points",1, &Pointing::cloudcb, this);
    skelsub_ = n_.subscribe( "/skeletons", 1, &Pointing::skelcb, this );
    // imgsub_ = n_.subscribe ( "/camera/rgb/image_color", 1, &Pointing::imgcb, this);
    detepub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("right_detected", 1);
    detepub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("left_detected", 1);
    count = 0;
    lastskelseq = 0;
    lastcloudseq = 0;
    skelmsg.header.seq = 0;
    pcloudmsg.header.seq = 0;
    // imgmsg.header.seq = 0;

    cout << "Init done!!!" << endl;
  }

  ~Pointing()
  {
    
  }

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
  /* \brief : 
   *
   *
   *
   *
   */


  void ProcessData(  body_msgs::Skeletons skels, sensor_msgs::PointCloud2 cloud )
  {
    if ( skels.skeletons.size() == 0 )
      return;
    

    cout << "Processing......." << endl;
    pcl::PointCloud<pcl::PointXYZ> fullcloud;
   

    //
    // cv_bridge::CvImageConstPtr imgptr;
    //imgptr = cv_bridge::toCvShare ( imgmsg, enc::BGR8 );
    
    extractPointedArea( cloud, skels.skeletons[0], 1);
    extractPointedArea( cloud, skels.skeletons[0], 0);
    
  }
  /*  \brief : 
   *
   *
   *
   *
   *
   */
  void extractPointedArea( sensor_msgs::PointCloud2  &cloudin, body_msgs::Skeleton skel, int is_right)
  {

    cout << "Extracting" << is_right << endl;
    std::vector<int> inds, maxcloud;
    int maxPointNumber = 0;
    //Eigen::Vector4f center;
    pcl::PointXYZ center, tmp;
    float distance = 0;

    pcl::PointCloud<pcl::PointXYZ> detected, fullcloud;
    sensor_msgs::PointCloud2 pointed_area;
    pcl::fromROSMsg( cloudin, fullcloud);


    if ( is_right )  
      center = pointToPclPoint(skel.right_hand.position);   
    else
      center = pointToPclPoint(skel.left_hand.position);   
      
    while (distance < 5.0 )
      {
	tmp = center;

	if ( is_right )
	  center = addVector(tmp, skel.right_elbow.position, skel.right_hand.position, 0.7);
	else
	  center = addVector(tmp, skel.left_elbow.position, skel.left_hand.position, 0.7);
	  
	distance += sqrt( (center.x - tmp.x) * (center.x - tmp.x) +
			  (center.y - tmp.y) * (center.y - tmp.y) +
			  (center.z - tmp.z) * (center.z - tmp.z));
	
	NNN(fullcloud, center, inds, .1);

	if ( inds.size() > maxPointNumber )
	  {
	    maxcloud = inds;
	    maxPointNumber = inds.size();
	  }
      }


    if (maxPointNumber == 0 ) 
      {
	cout << "Can not detect" << endl;

      }
    else
      {
	getSubCloud(fullcloud, maxcloud, detected);
      }

    pcl::toROSMsg( detected, pointed_area );
    pointed_area.header = cloudin.header;

    if ( is_right )
      detepub_[0].publish( pointed_area );
    else
      detepub_[1].publish( pointed_area );

  }

  /* \brief : Callback function for each time Poincloud is received
   *
   *
   *
   */
  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &fullcloud)
  {
    pcloudmsg = *fullcloud;
    messageSync();
  }
  
  void skelcb(const body_msgs::SkeletonsConstPtr &skels)
  {
    skelmsg = * skels;
    messageSync();
  }

  void imgcb( const sensor_msgs::ImageConstPtr & img)
  {
    imgmsg = img;
    messageSync();
  }

    

};


  int main ( int argc, char ** argv)
  {
    ros::init(argc, argv, "pointing_area");
    ros::NodeHandle n;
    Pointing detector;
    ros::spin();
    return 0;
  }
