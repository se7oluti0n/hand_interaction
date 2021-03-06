/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <body_msgs/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <nnn/nnn.hpp>
#include <pcl_tools/segfast.hpp>


#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <body_msgs/Skeletons.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b TimeEvaluator is a nifty function to make benchmarking programs much simpler
 * \author Garratt Gallagher
 */
class TimeEvaluator{
   timeval tmark;
   std::vector<double> times;
   std::vector<std::string> eventnames;
   std::string name;  //name is used to identify the evaluator as a whole


public:
   /** \brief Constructor. Initializes timer, so this is the first time record. */
   TimeEvaluator(std::string _name="Time Evaluator"){
      name=_name;
      //be default the clock starts running when TimeEvaluator is initialized

      gettimeofday(&tmark, NULL);

   }

   /** \brief records this time, optionally with a user specified name. difference is calculated from last mark call */
   void mark(std::string _name=""){
      //Give the event a name:
      if(_name.size())
         eventnames.push_back(_name);
      else{
         int count=eventnames.size();
         char tname[10];
         sprintf(tname,"E%d",count);
         eventnames.push_back(std::string(tname));
      }
      //record the time since last event
      struct timeval tv;
      gettimeofday(&tv, NULL);
      times.push_back((double)(tv.tv_sec-tmark.tv_sec) + (tv.tv_usec-tmark.tv_usec)/1000000.0);
   }

   /** \brief print out all the time differences */
   void print(){
      std::cout<<name;
      for(uint i=0;i<times.size();++i)
         std::cout<<"  "<<eventnames[i]<<": "<< std::setprecision (5) << times[i];
      std::cout<<std::endl;
   }




};



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
/*
template <typename PointT>
void getSubCloud(pcl::PointCloud<PointT> &cloudin,  std::vector<int> &ind, pcl::PointCloud<PointT> &cloudout,bool use_positive=true){
   pcl::ExtractIndices<PointT> extract;
   // Extract the inliers
   extract.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> > (cloudin));
   extract.setIndices (boost::make_shared<std::vector<int> > (ind));
   extract.setNegative (!use_positive);
   extract.filter (cloudout);
 //    ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);
 
 }*/
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
pcl::PointXYZ addVector(const Eigen::Vector4f &_C, geometry_msgs::Point A, geometry_msgs::Vector3 B, double scale){
  Eigen::Vector4f C=_C;
   C(0)+=scale*(B.x-A.x);
   C(1)+=scale*(B.y-A.y);
   C(2)+=scale*(B.z-A.z);
   return eigenToPclPoint(C);
}


bool isJointGood(body_msgs::SkeletonJoint &joint){
   if(joint.confidence < 0.5)
      return false;
   else
      return true;
}

void getEigens(body_msgs::Hand &h){
   pcl::PointCloud<pcl::PointXYZ> cloud;

   Eigen::Vector4f centroid, direction,armvector;
   pcl::fromROSMsg(h.handcloud,cloud);
   EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
     EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
     Eigen::Matrix3f cov;
     pcl::compute3DCentroid (cloud, centroid);
     pcl::computeCovarianceMatrixNormalized(cloud,centroid,cov);
     pcl::eigen33 (cov, eigen_vectors, eigen_values);
     direction(0)=eigen_vectors (0, 2);
     direction(1)=eigen_vectors (1, 2);
     direction(2)=eigen_vectors (2, 2);
     armvector(0)=h.arm.x; armvector(1)=h.arm.y; armvector(2)=h.arm.z;
     flipvec(armvector,centroid,direction);
     //printf("Eigenvalues: %.02f, %.02f \n",eigen_values(0)/eigen_values(1),eigen_values(1)/eigen_values(2));
     if(eigen_values(1)/eigen_values(2) < .4)
        h.state=std::string("closed");
     else
        h.state=std::string("open");
     //eigen eigen_values(1)/eigen_values(2) < .4 means closed fist, unless you are pointing at the kinect

//     //make polygon
//     geometry_msgs::Polygon p;
//     p.points.push_back(eigenToMsgPoint32(centroid));
//     p.points.push_back(eigenToMsgPoint32(centroid+direction));
//     pmap.polygons.push_back(p);
//     pmap.header=h.handcloud.header;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief grabs the correct portion of the point cloud to get the hand cloud
  * \param the resultant Hand message with the location of the hand and arm already added.  This message is filled out further in this function
  * \param fullcloud the full point cloud from the kinect
  */
void getHandCloud(body_msgs::Hand &hand, sensor_msgs::PointCloud2 &fullcloud){
   pcl::PointCloud<pcl::PointXYZ> handcloud,cloudin;
   //convert to pcl cloud
   pcl::fromROSMsg(fullcloud,cloudin);

   std::vector<int> inds;
   Eigen::Vector4f handcentroid;
   pcl::PointXYZ handpos;
   PointConversion(hand.palm.translation,handpos);  //updating estimate of location of the hand

   cout << cloudin.points[640 * 238 + 318].x << " " << cloudin.points[640 * 238 + 318].y << endl;
   cout << cloudin.points[640 * 238 + 308].x << " " << cloudin.points[640 * 238 + 328].x << endl;
   // cout << cloudin.points[319].x << " " << cloudin.points[319].y << endl;
   
   // printf("got hand %.02f, %02f, %02f ",handpos.x, handpos.y,handpos.z);
   //find points near the skeletal hand position
   NNN(cloudin,handpos,inds, .1);

   //Iterate the following:
   //    find centroid of current cluster
   //    push the cluster slightly away from the arm
   //    search again around the centroid to redefine our cluster

   for(int i=0; i<3;i++){
      pcl::compute3DCentroid(cloudin,inds,handcentroid);
      handpos=addVector(handcentroid,hand.arm,hand.palm.translation,.05);
      NNN(cloudin,handpos,inds, .1);
   }

   //save this cluster as a separate cloud.
   getSubCloud(cloudin,inds,handcloud);

   //convert the cloud back to a message
   pcl::toROSMsg(handcloud,hand.handcloud);
   PointConversion(handpos,hand.palm.translation);

   //add other hand message stuff:
   hand.state="unprocessed";
   getEigens(hand);
   std::cout<<hand.state<<std::endl;
   hand.thumb=-1; //because we have not processed the hand...
   hand.stamp=fullcloud.header.stamp;
   hand.handcloud.header=fullcloud.header;


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief converts a skeleton + cloud into a hands message, by calling getHandCloud
  * \param skel the skeleton who's hands we need to find
  * \param cloud the full point cloud from the kinect
  * \param handsmsg the resultant Hands message
  */
void getHands(body_msgs::Skeleton &skel, sensor_msgs::PointCloud2 &cloud, body_msgs::Hands &handsmsg ){
   //first hand:
   // printf("Torso: %f %f %f\n", skel.torso.position.x, skel.torso.position.y, skel.torso.position.z);
   if(isJointGood(skel.left_hand)){
      body_msgs::Hand lhand;
      lhand.arm=skel.left_elbow.position;
      lhand.palm=pointToTransform(skel.left_hand.position);
      getHandCloud(lhand,cloud);
      handsmsg.hands.push_back(lhand);
      handsmsg.hands.back().left=true;
   }

   if(isJointGood(skel.right_hand)){
      body_msgs::Hand rhand;
      rhand.arm=skel.right_elbow.position;
      rhand.palm=pointToTransform(skel.right_hand.position);
      getHandCloud(rhand,cloud);
      handsmsg.hands.push_back(rhand);
      handsmsg.hands.back().left=false;
   }
   if(isJointGood(skel.left_hand) ||  isJointGood(skel.right_hand))
      std::cout<<std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b HandDetector is the main ROS communication class, and its function is just to tye things together.
 * \author Garratt Gallagher
 */
class HandDetector
{

private:
  ros::NodeHandle n_;
  ros::Publisher cloudpub_[2],handspub_, pca1pub_, pca2pub_, pca3pub_, xpub_, ypub_, zpub_;
  ros::Subscriber cloudsub_,skelsub_;
  std::string fixedframe;
  //the latest two messages we have received:
  body_msgs::Skeletons skelmsg;
  sensor_msgs::PointCloud2 pcloudmsg;
  int lastskelseq, lastcloudseq;
  mapping_msgs::PolygonalMap pca1pmap, xpmap, ypmap, zpmap;
  mapping_msgs::PolygonalMap pca2pmap;
  mapping_msgs::PolygonalMap pca3pmap;


public:

  HandDetector()
{
   handspub_ = n_.advertise<body_msgs::Hands> ("hands", 1);
   pca1pub_ = n_.advertise<mapping_msgs::PolygonalMap> ("PCA1st", 1);
   pca2pub_ = n_.advertise<mapping_msgs::PolygonalMap> ("PCA2nd", 1);
   pca3pub_ = n_.advertise<mapping_msgs::PolygonalMap> ("PCA3rd", 1);

   xpub_ = n_.advertise<mapping_msgs::PolygonalMap> ("x_axis", 1);
   ypub_ = n_.advertise<mapping_msgs::PolygonalMap> ("y_axis", 1);
   zpub_ = n_.advertise<mapping_msgs::PolygonalMap> ("z_axis", 1);
   
   cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_fullcloud", 1);
   cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_fullcloud", 1);
   cloudsub_=n_.subscribe("/camera/rgb/points", 1, &HandDetector::cloudcb, this);
   skelsub_=n_.subscribe("/skeletons", 1, &HandDetector::skelcb, this);
   lastskelseq=0;
   lastcloudseq=0;
   skelmsg.header.seq=0;
   pcloudmsg.header.seq=0;
}

  /** \brief This functions tries to sync the skeleton and point cloud messages */
  void messageSync(){
     //don't even consider it if the sequence numbers have not changed
     if(skelmsg.header.seq == lastskelseq || pcloudmsg.header.seq == lastcloudseq)
        return;

     double tdiff = (skelmsg.header.stamp-pcloudmsg.header.stamp).toSec();
     //At 30 hz, assume that the timing will be less than 15ms apart
     if(fabs(tdiff) < .15){
        lastskelseq=skelmsg.header.seq;
        lastcloudseq=pcloudmsg.header.seq;
        processData(skelmsg,pcloudmsg);
     }
  }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Gets the direction of the hand.  Useful for determining where the hand is pointing
    */
  void getEigens(const body_msgs::Hand &h){
     pcl::PointCloud<pcl::PointXYZ> cloud;

     Eigen::Vector4f centroid, direction,armvector;
     pcl::fromROSMsg(h.handcloud,cloud);
     EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
     EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
       Eigen::Matrix3f cov;
       pcl::compute3DCentroid (cloud, centroid);
       pcl::computeCovarianceMatrixNormalized(cloud,centroid,cov);
       pcl::eigen33 (cov, eigen_vectors, eigen_values);
       direction(0)=eigen_vectors (0, 0);
       direction(1)=eigen_vectors (1, 0);
       direction(2)=eigen_vectors (2, 0);
       armvector(0)=h.arm.x; armvector(1)=h.arm.y; armvector(2)=h.arm.z;
       flipvec(armvector,centroid,direction);
       //printf("Eigenvalues: %.02f, %.02f \n",eigen_values(0)/eigen_values(1),eigen_values(1)/eigen_values(2));

       //eigen eigen_values(1)/eigen_values(2) < .4 means closed fist, unless you are pointing at the kinect

       //make polygon
       Eigen::Vector3f  arm1,yvec, xxis, yxis, zxis;
       arm1[0] = h.palm.translation.x - h.arm.x;
       arm1[1] = h.palm.translation.y - h.arm.y;
       arm1[2] = h.palm.translation.z - h.arm.z;

       yvec[0] = 0;
       yvec[1] = 1;
       yvec[2] = 0;
       
       zxis = arm1;
       xxis = yvec.cross(arm1);
        double cos = arm1.dot(zxis) / sqrt( arm1.norm() * zxis.norm() );
	if ( cos < 0 ) xxis = - xxis;
	
	
	yxis = zxis.cross(xxis);
	zxis.normalize();
	yxis.normalize();
	xxis.normalize();
	
	direction(0)= zxis[0];
	direction(1)= zxis[1];
	direction(2)= zxis[2];

       geometry_msgs::Polygon p1;
       p1.points.push_back(eigenToMsgPoint32(centroid));
       p1.points.push_back(eigenToMsgPoint32(centroid+direction));
       pca1pmap.polygons.push_back(p1);
       pca1pmap.header=h.handcloud.header;


       /*direction(0)=eigen_vectors (0, 1);
       direction(1)=eigen_vectors (1, 1);
       direction(2)=eigen_vectors (2, 1);
       */
       direction(0)= xxis[0];
	direction(1)= xxis[1];
	direction(2)= xxis[2];
       geometry_msgs::Polygon p2;
       p2.points.push_back(eigenToMsgPoint32(centroid));
       p2.points.push_back(eigenToMsgPoint32(centroid+direction));
       pca2pmap.polygons.push_back(p2);
       pca2pmap.header=h.handcloud.header;

       /*direction(0)=eigen_vectors (0, 2);
       direction(1)=eigen_vectors (1, 2);
       direction(2)=eigen_vectors (2, 2);
       */
       direction(0)= yxis[0];
       direction(1)= yxis[1];
       direction(2)= yxis[2];
       geometry_msgs::Polygon p3;
       p3.points.push_back(eigenToMsgPoint32(centroid));
       p3.points.push_back(eigenToMsgPoint32(centroid+direction));
       pca3pmap.polygons.push_back(p3);
       pca3pmap.header=h.handcloud.header;

       geometry_msgs::Polygon px, py, pz;
       Eigen::Vector4f origin, axis;
       origin(0)=0;
       origin(1)=0;
       origin(2)=0;
       
       axis(0) = 1;
       axis(1) = 0;
       axis(2) = 0;
       px.points.push_back(eigenToMsgPoint32(origin));
       px.points.push_back(eigenToMsgPoint32(origin+axis));
       xpmap.polygons.push_back(px);
       xpmap.header = h.handcloud.header;


       axis(0) = 0;
       axis(1) = 1;
       axis(2) = 0;
       py.points.push_back(eigenToMsgPoint32(origin));
       py.points.push_back(eigenToMsgPoint32(origin+axis));
       ypmap.polygons.push_back(py);
       ypmap.header = h.handcloud.header;



       axis(0) = 0;
       axis(1) = 0;
       axis(2) = 1;
       pz.points.push_back(eigenToMsgPoint32(origin));
       pz.points.push_back(eigenToMsgPoint32(origin+axis));
       zpmap.polygons.push_back(pz);
       zpmap.header = h.handcloud.header;
			   
			   
  }


  /** \brief This functions is called when a skeleton message and point cloud are synchronized */
  void processData(body_msgs::Skeletons skels, sensor_msgs::PointCloud2 cloud){
     //nothing to do if multiple skeletons...
     if(skels.skeletons.size()==0)
        return;
     //TODO: maybe pick the closest skeleton?
     body_msgs::Hands hands;
     getHands(skels.skeletons[0],cloud,hands);

     pca1pmap.polygons.clear();
     pca2pmap.header=hands.header;

     pca2pmap.polygons.clear();
     pca2pmap.header=hands.header;

     pca3pmap.polygons.clear();
     pca3pmap.header=hands.header;

     xpmap.polygons.clear();
     xpmap.header = hands.header;

     ypmap.polygons.clear();
     ypmap.header = hands.header;

     zpmap.polygons.clear();
     zpmap.header = hands.header;
     
     // Publish hands
     for(uint i=0;i<hands.hands.size();i++){
        if(hands.hands[i].left)
           cloudpub_[0].publish(hands.hands[i].handcloud);
        else
           cloudpub_[1].publish(hands.hands[i].handcloud);

	getEigens(hands.hands[i]);
     }
     if(hands.hands.size())
       {
	 handspub_.publish(hands);
	 pca1pub_.publish(pca1pmap);

	 pca2pub_.publish(pca2pmap);

	 pca3pub_.publish(pca3pmap);

	 xpub_.publish(xpmap);
	 ypub_.publish(ypmap);
	 zpub_.publish(zpmap);
       }
  }

  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
     pcloudmsg=*scan;
     //cout << "W & H: " << pcloudmsg.width << " " << pcloudmsg.height << endl;
     messageSync();
  }

  void skelcb(const body_msgs::SkeletonsConstPtr &skels){
     skelmsg=*skels;
     //  printf("skel callback tdiff = %.04f \n",(skelmsg.header.stamp-pcloudmsg.header.stamp).toSec());
     messageSync();
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detector");
  ros::NodeHandle n;
  HandDetector detector;
  ros::spin();
  return 0;
}
