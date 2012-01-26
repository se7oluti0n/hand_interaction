#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <body_msgs/Skeletons.h>
#include <sensor_msgs/PointCloud.h>
#include <body_msgs/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <mapping_msgs/PolygonalMap.h>
#include <std_msgs/Bool.h>
#include <pcl_tools/pcl_utils.h>
#include <hand_interaction/Pointing.h>
#include <nnn/nnn.hpp>

//#include <pcl_tools/segfast.hpp>

#include <pcl/common/impl/angles.hpp>
#include <pcl/common/transform.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <cv_bridge/cv_bridge.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pthread.h>

#include "svm.h"
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

char key;
bool object_selected = false;
bool started = false; 
int detected_num = 0;

void onMouseClick( int event, int x, int y, int flag, void * param);
struct arms
{
  body_msgs::SkeletonJoint left_hand;
  body_msgs::SkeletonJoint left_elbow;
  body_msgs::SkeletonJoint right_hand;
  body_msgs::SkeletonJoint right_elbow;
};

struct feature
{
  struct svm_node ft[250];
};

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

vector<pcl::ModelCoefficients> coeffs;
//vector<vtkSmartPointer< vtkPolyData >> circles; 
boost::mt19937 gen;
pcl::visualization::CloudViewer viewer("Cloud Viewer");
pcl::PointCloud<pcl::PointXYZ> cylinder_cloud;
pcl::PointXYZ ff;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Features Visual Viewer"));


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

geometry_msgs::Point32 PointToMsgPoint32(const geometry_msgs::Point p64){
	geometry_msgs::Point32 p;
	p.x=p64.x; p.y=p64.y; p.z=p64.z;
	return p;
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

void visual (pcl::visualization::PCLVisualizer& viewera)
  {
    //string id;
    //char c='a';
   
    /*  viewer->removePointCloud();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    viewer->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();
    
    */

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cylinder_cloud.makeShared(), 0, 255, 0);
    viewera.removePointCloud("cylinder");
    viewera.addPointCloud<pcl::PointXYZ> (cylinder_cloud.makeShared(), single_color,  "cylinder");
    viewera.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cylinder");
    viewera.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 0, "cylinder");

    viewera.removeShape("sphere");
    viewera.addSphere (ff, 0.01, 0.5, 0.5, 0.0, "sphere");
    viewera.setShapeRenderingProperties(  pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 1.0, "sphere");
    
    //id = "circle";
    
    /*for (int i = 0; i < coeffs.size(); i++ )
      {
	id+=c;
	viewera.removeShape(id);
	viewera.addCylinder(coeffs[i], id); 
	viewera.setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, id);
      }
     */
    //viewer->spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
} 


struct HandSaver
{
 private:
  ros::NodeHandle n_;
  ros::Subscriber cloudsub_, skelsub_, imgsub_, fCloudsub_;
  //ros::Publisher pointingpub_, fingerpub_;
    ros::Publisher detepub_[2], laserpub_;
  sensor_msgs::PointCloud2 pcloudmsg, fcloudmsg;
  body_msgs::Skeletons skelmsg;
  sensor_msgs::Image imgmsg;

  geometry_msgs::Point finger_point;
  
  std::vector<struct feature> frame;
  struct feature x;
  int max_nr_attr;
  
  struct svm_model *model;
  int predict_probability;
  stringstream filename;

  fstream pointingError;
  std::string name, foldername;
  int count, pos, neg ;
  int save;
  int lastskelseq, lastcloudseq, lastimgseq, lastfcloudseq;
  timeval t0;

  int svm_type, nr_class;
  double *prob_estimates;


  hand_interaction::Pointing pointingmsg;
   mapping_msgs::PolygonalMap laserpmap;

  // Networking variables
  /*
  int sock, connected, bytes_received, istrue;
  struct sockaddr_in server_addr, client_addr;
  socklen_t sin_size;
  struct hostent *host;
  
  */

  Mat cameraMatrix, distCoeffs, rotation_vector, translation_vector;
  string input_file;
public:

  cv::Mat kinectImg;
  pcl::PointXYZ objectPosition;
  //vector<Point2f> ipPoints, kinectPoints;
  pcl::PointCloud<pcl::PointXYZ> objectCloud, objectCloud2;
  int maxx,maxy, minx, miny, handx, handy;
  int isPointing;
  
  HandSaver(std::string name_, int saveChoice, string folder, int onl = 1, int prob =  1):foldername(folder), name(name_), predict_probability(prob), max_nr_attr(64), save(saveChoice)
    {
      fCloudsub_ = n_.subscribe("/camera/rgb/points",1, &HandSaver::fcloudcb, this);
      cloudsub_ = n_.subscribe("hand1_fullcloud", 1, &HandSaver::cloudcb, this);      
      skelsub_ = n_.subscribe("/skeletons", 1, &HandSaver::skelcb, this);
      imgsub_  = n_.subscribe("/camera/rgb/image_color", 1, &HandSaver::imgcb, this);

      detepub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("right_detected", 1);
      laserpub_ = n_.advertise<mapping_msgs::PolygonalMap> ("laser", 1);

      count = 0;
      pos = 0;
      neg = 0;
      t0 = g_tick();
      lastskelseq = 0;
      lastcloudseq = 0;
      lastimgseq = 0;
      lastfcloudseq = 0;
      skelmsg.header.seq = 0;
      pcloudmsg.header.seq = 0;
      imgmsg.header.seq = 0;
      fcloudmsg.header.seq = 0;
      string cmd;
      cmd = "mkdir " + foldername;
      system(cmd.c_str());
      
      if ((model = svm_load_model(name.c_str())) == 0 )
      {
	cerr << "Can't open input file " << name << endl; 
	exit(1);
      }

     
      //   x = ( struct svm_node *) malloc ( 300 * sizeof(struct svm_node));

      	if(predict_probability)
	{
		if(svm_check_probability_model(model)==0)
		{
		  cerr << "Model does not support probabiliy estimates\n" << endl;
			exit(1);
		}
	}
	else
	{
		if(svm_check_probability_model(model)!=0)
		  cout << "Model supports probability estimates, but disabled in prediction.\n" << endl;
	}

	svm_type=svm_get_svm_type(model);
	nr_class=svm_get_nr_class(model);
	prob_estimates = (double *) malloc(nr_class*sizeof(double));
	isPointing = 0;
	
	input_file = "ExtrinsicParameters.yml";
	load_camera_parameters( input_file, cameraMatrix, distCoeffs, rotation_vector, translation_vector);
	
	//	pointingError.open("pointingError.csv", fstream::app);
	pointingError.open("pointingError.csv", fstream::out);
	cout << "Initilization complete." << endl;
	
	//	initNetwork();
	
	
    }

  void load_camera_parameters( string input_file, Mat& cameraMat, Mat& distCo, Mat& rvec, Mat & tvec)
  {
    FileStorage fs(input_file, FileStorage::READ);
    fs["camera_matrix"] >> cameraMat;
    fs["distortion_coefficients"] >> distCo;
    fs["rotation_vector"] >> rvec;
    fs["translation_vector"] >> tvec;
    
  }

  sensor_msgs::Image getImagemsg()
  {
    return imgmsg;
  }


  ~HandSaver()
  {
    // free(x);
    svm_free_and_destroy_model(&model);
    if ( predict_probability )
    free(prob_estimates);
    //    close(sock);
    pointingError.close();
  }



    pcl::PointXYZ draw_sample ( pcl::PointXYZ center, float error )
  {
    float z = center.z;
    float min = z - error / 2;
    float max = z + error / 2;

    //boost::random::mt19937 rng;         // produces randomness out of thin air
    // see pseudo-random number generators
    boost::uniform_real<> u(min,max);
    // distribution that maps to 1..6
    // see random number distributions
    float x = u(gen);                   // simulate rolling a die
    
    return pcl::PointXYZ( center.x, center.y, x );

  } 
  void resample( pcl::PointCloud<pcl::PointXYZ> & cloudin, pcl::PointCloud<pcl::PointXYZ> & cloudout, arms  &armin )
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


  // \brief This function tries to sync the skeleton and point cloud messages 
  void messageSync()
  {
    
    if ( skelmsg.header.seq == lastskelseq || pcloudmsg.header.seq == lastcloudseq || imgmsg.header.seq == lastimgseq || fcloudmsg.header.seq == lastfcloudseq)
      {
	//cout << "Not enough data" << endl;
      return;
      }
    double tdiff = (skelmsg.header.stamp - pcloudmsg.header.stamp).toSec();
    double tdiff2 =  (imgmsg.header.stamp - pcloudmsg.header.stamp).toSec();
    double tdiff3 =  (imgmsg.header.stamp - skelmsg.header.stamp).toSec();
    double tdiff4 = (skelmsg.header.stamp - fcloudmsg.header.stamp).toSec();
    
    if (fabs(tdiff) < .15 && fabs(tdiff2) < .15 && fabs(tdiff3) < .15 && fabs(tdiff4) < .15 ){
      lastskelseq = skelmsg.header.seq;
      lastcloudseq = pcloudmsg.header.seq;
      lastimgseq = imgmsg.header.seq;
      lastfcloudseq = fcloudmsg.header.seq;
      ProcessData(skelmsg, pcloudmsg, fcloudmsg);
     }
    // else
    //cout << "Delay between data" << endl;


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

    void extractArm( body_msgs::Skeleton &skel, arms &a)
  {
    a.left_hand = skel.left_hand;
    a.right_hand = skel.right_hand;
    a.left_elbow = skel.left_elbow;
    a.right_elbow = skel.right_elbow;
  }

 int check_3d_position(int x, int y, pcl::PointXYZ &p)
  {
     pcl::PointCloud<pcl::PointXYZ> cloud;
     pcl::fromROSMsg( fcloudmsg, cloud);

     p = cloud.at(x, y);
     // p.x = - (p.x);
     
     cout << "2D: " << x << " " << y << "  >>  3D: " << p.x << " " << " " << p.y << " " << p.z << endl;
     if (p.z == NULL )
       return 0;
     else 
       return 1;
  }
  void extractFeatures( pcl::PointCloud<pcl::PointXYZ> cloud2, arms &skel, hand_interaction::Pointing &pointing )
  {
    pcl::PointCloud<pcl::PointXYZ>  output, cloud, fingertip;
   
     EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
     EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
     Eigen::Matrix3f cov;
     Eigen::Vector4f centroid ,armvector;
  
     Eigen::Vector3f z_axis, y_axis, x_axis, origin;
     Eigen::Affine3f transformation;

     Eigen::Vector3f right_arm, yvector;
     float arm_length;
     //viewer.showCloud(cloud2);
     sensor_msgs::PointCloud2 trans1, trans2;
      
     
     resample(cloud2, cloud, skel);
     right_arm[0] = skel.right_hand.position.x - skel.right_elbow.position.x;
     right_arm[1] = skel.right_hand.position.y - skel.right_elbow.position.y;
     right_arm[2] = skel.right_hand.position.z - skel.right_elbow.position.z;

     arm_length = right_arm.norm();
     pcl::compute3DCentroid (cloud, centroid);

     origin [ 0 ] = centroid(0);
     origin [ 1 ] = centroid(1);
     origin [ 2 ] = centroid(2);
     
     /*origin [ 0 ] = skel.right_hand.position.x;
     origin [ 1 ] = skel.right_hand.position.y;
     origin [ 2 ] = skel.right_hand.position.z;*/
     //pcl::computeCovarianceMatrixNormalized(cloud,centroid,cov);
     //pcl::eigen33 (cov, eigen_vectors, eigen_values);

   
     z_axis[0] = right_arm[0];//eigen_vectors( 0, 2);
     z_axis[1] = right_arm[1];//eigen_vectors( 1, 2);
     z_axis[2] = right_arm[2];// eigen_vectors( 2, 2);
   
     yvector[0]  = 0;
     yvector[1]  = 1;
     yvector[2]  = 0; 
    /*  y_axis[0] = eigen_vectors( 0, 0);
    y_axis[1] = eigen_vectors( 1, 0);
    y_axis[2] = eigen_vectors( 2, 0);

    origin [ 0 ] = centroid[0];
    origin [ 1 ] = centroid[1];
    origin [ 2 ] = centroid[2];
    */

     x_axis = yvector.cross(right_arm);
     double cos = right_arm.dot(z_axis) / sqrt( right_arm.norm() * z_axis.norm() );
     if ( cos < 0 ) x_axis = - x_axis;
    
     y_axis = z_axis.cross(x_axis);
   
     x_axis.normalize();
     y_axis.normalize();
     z_axis.normalize();



   //    double cos = right_arm.dot(z_axis) / sqrt( right_arm.norm() * z_axis.norm() );
   //if ( cos < 0 ) z_axis = - z_axis;
   // cout << " tan: "<< tann ;
    
   /*y_axis[0] = eigen_vectors( 0, 0);
   y_axis[1] = eigen_vectors( 1, 0);
   y_axis[2] = eigen_vectors( 2, 0);
   */
 


     pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_axis, z_axis, origin, transformation);

     pcl::getTransformedPointCloud (cloud, transformation, output);
   
   // if ( output.is_dense)
   
   // pcl::toROSMsg( output, trans1 );
   //   trans1.header = pcloudmsg.header;
   //handpub_[0].publish( trans1 );
   
   

   pcl::PointXYZ min_pt, max_pt;

   pcl::getMinMax3D( output, min_pt, max_pt);

   
   //float r = ( max_pt.y - min_pt.y )* ( max_pt.y - min_pt.y) +  ( min_pt.x - max_pt.x) * ( min_pt.x - max_pt.x )  ;
   // r /= 4;

      
   /*float r1 = max_pt.y * max_pt.y + max_pt.x * max_pt.x  ;
    float r2 = min_pt.y * min_pt.y + min_pt.x * min_pt.x;
    float r3 = max_pt.y * max_pt.y + min_pt.x * min_pt.x;
    float r4 = min_pt.y * min_pt.y + max_pt.x * max_pt.x;

    
    
    float r = r1 > r2?r1:r2;
    r = r > r3?r:r3;
    r = r > r4?r:r4;
   */
   //cout << "R: " << r << endl;
    float r = 0.4 * arm_length;
    
    float offset_r = ( r * r ) / 4.999;
    origin[2] = min_pt.z;
    
    float offset_z = 0.7 * arm_length / 5.999;
    const double PI = 3.14159266;
    float offset_a = PI / 3.999;

    float histogram[240];
    //  cout << "Max length : " << max_pt.z - min_pt.z << endl; 
    for ( int i = 0; i < 240; i++ )
      
      {
	histogram[i] = 0.0;
	//	cout << histogram[i];
      }

    // Visualation
    
    float zzz = min_pt.z;
    cylinder_cloud.points.clear();
    for (int i = 0; i < 7; i++, zzz+= offset_z )
      {

	float orr = r / 5.0;
	float radiuss = orr;
	for ( int j = 0; j < 5; j++, radiuss += orr)
	  {
	    for (float angle(0.0); angle <= 360.0; angle += 5.0)
	      {
		pcl::PointXYZ basic_point;
		basic_point.x =  radiuss * cosf (pcl::deg2rad(angle));
		basic_point.y =  radiuss * sinf (pcl::deg2rad(angle));
		basic_point.z = zzz;
		cylinder_cloud.points.push_back(basic_point); 
	      }
	  }

	orr = r / 25.0;
	 for (float angle(0.0); angle <= 360.0; angle += 45.0)
	      {
		for (radiuss = 0; radiuss <= r; radiuss+= orr)
		  {
		pcl::PointXYZ basic_point;
		basic_point.x =  radiuss * cosf (pcl::deg2rad(angle));
		basic_point.y =  radiuss * sinf (pcl::deg2rad(angle));
		basic_point.z = zzz;
		cylinder_cloud.points.push_back(basic_point); 
		  }
	      }

	
      }
    cylinder_cloud.width = cylinder_cloud.points.size();
    cylinder_cloud.height = cylinder_cloud.points.size();


    if (output.is_dense)
      {
	viewer.showCloud(output.makeShared());
	viewer.runOnVisualizationThreadOnce(visual);
      }
    /*************************************************/

    float fingertip_z = max_pt.z - (max_pt.z - min_pt.z) / 6;
    
     for (unsigned int i = 0; i < output.points.size(); i++ )
      {
	if ( output.points[i].z > fingertip_z ) 
	  fingertip.points.push_back(output.points[i]);
	

	int zz = floor (( output.points[i].z - origin[2] ) / offset_z);
	zz = ( zz > 5? 5 : zz );
	int pp = (int) (( output.points[i].y * output.points[i].y  + output.points[i].x * output.points[i].x ) / offset_r );
	pp = ( pp > 4? 4 : pp );
	int aa = (int) (( PI + atan2(output.points[i].y, output.points[i].x)) / offset_a );

        int index = zz * 40 + pp * 8 + aa % 8;
	//if ( index < 0 || index > 239 ) cout << "Out of range!!" << " " << zz << " " << pp << " " << aa << endl;
	
	histogram[ index  ] += 1.0;

      }

     Eigen::Vector4f fingertip_centroid;
    
    

     pcl::PointXYZ local_centroid, global_centroid;

     pcl::compute3DCentroid( fingertip, fingertip_centroid);
     local_centroid = eigenToPclPoint(fingertip_centroid);
     ff = local_centroid;
     Eigen::Affine3f inverse_transformation;

     pcl::getInverse( transformation, inverse_transformation);
     global_centroid = pcl::transformXYZ ( inverse_transformation, local_centroid);
     
     PointConversion(global_centroid, finger_point);
     //fingerpub_.publish(
     
     pointing.fingertip = finger_point;
     
     for ( int i = 0; i < 240; i++ )
    {
      histogram[i] /= (float) output.points.size();
      x.ft[i].index = i+1;
      x.ft[i].value = histogram[i];
      // tcout << histogram[i] << endl;
    }
     x.ft[240].index = -1;
     averageFrame();

  }

    void averageFrame()
  {
    std::vector<struct feature>::iterator it;
    it = frame.begin();
    it = frame.insert(it, x);
    if (frame.size() > 9 ) frame.pop_back(); 
    for (int j = 0; j < 240; j++ )
      { 
	float sum = 0.0;
	for (int i = 0; i < frame.size(); i++ )
	  {
	    sum += frame[i].ft[j].value;
	  }
	sum /= frame.size();
	x.ft[j].value = sum;
      }

  }

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

  /*  \brief : 
   *
   *
   *
   *
   *
   */
  int extractPointedArea( sensor_msgs::PointCloud2  &cloudin, body_msgs::Skeleton skel, int is_right,  pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt, pcl::PointXYZ &vocuc, pcl::PointXYZ &pointPos, pcl::PointXYZ &detected_center  )
  {

    //cout << "Extracting....." << is_right << endl;
    std::vector<int> inds, maxcloud;
    int maxPointNumber = 0;
    //Eigen::Vector4f center;
    pcl::PointXYZ center, tmp;
    float density_threshold, search_radius = 0.1;
    float distance = 0;

    pcl::PointCloud<pcl::PointXYZ> detected, fullcloud;
    sensor_msgs::PointCloud2 pointed_area;
    pcl::fromROSMsg( cloudin, fullcloud);


    if ( is_right )  
      PointConversion(pointingmsg.fingertip, center);   
    else
      center = pointToPclPoint(skel.left_hand.position);   
      
    while (distance < 5.0 )
      {
	tmp = center;

	if ( is_right )
	  center = addVector(tmp, skel.right_elbow.position, skel.right_hand.position, 0.5);
	else
	  center = addVector(tmp, skel.left_elbow.position, skel.left_hand.position, 0.5);
	
	if (center.z < -0.1 ) break;
	
	distance = sqrt( (center.x - pointingmsg.fingertip.x) * (center.x - pointingmsg.fingertip.x) +
			  (center.y - pointingmsg.fingertip.y) * (center.y - pointingmsg.fingertip.y) +
			  (center.z - pointingmsg.fingertip.z) * (center.z - pointingmsg.fingertip.z));

	
	
	NNN(fullcloud, center, inds, search_radius);
	pointPos = center;

	if ( inds.size() > maxPointNumber )
	  {
	    maxcloud = inds;
	    maxPointNumber = inds.size();
	    if (maxPointNumber > 200 ) break;
	  }

	
      }


    //cout << "max point number : " << maxPointNumber << endl;
    int is_detected = 1;

    Eigen::Vector4f  arm1, goc, huvo;
    arm1(0) = skel.right_hand.position.x - skel.right_elbow.position.x;
    arm1(1) = skel.right_hand.position.y - skel.right_elbow.position.y;
    arm1(2) = skel.right_hand.position.z - skel.right_elbow.position.z;
    
    goc(0) = pointingmsg.fingertip.x;
    goc(1) = pointingmsg.fingertip.y;
    goc(2) = pointingmsg.fingertip.z;
    
    
    arm1*= 20; 
    huvo = goc + arm1;

     geometry_msgs::Polygon p1;
     p1.points.push_back(PointToMsgPoint32( pointingmsg.fingertip));
    
     

     if (maxPointNumber == 0 ) 
      {
	//	cout << "Can not detect" << endl;
	is_detected = 0;
	p1.points.push_back(eigenToMsgPoint32(huvo));
	vocuc.x = huvo(0);
	vocuc.y = huvo(1);
	vocuc.z = huvo(2);
      }
    else
      {
	getSubCloud(fullcloud, maxcloud, detected);
	pcl::getMinMax3D( detected , min_pt, max_pt);
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid (detected, centroid);
	p1.points.push_back(eigenToMsgPoint32(centroid));
	detected_center.x = centroid(0);
	detected_center.y = centroid(1);
	detected_center.z = centroid(2);
      }

    pcl::toROSMsg( detected, pointed_area );
    pointed_area.header = cloudin.header;

    laserpmap.polygons.push_back(p1);
    laserpmap.header=cloudin.header;
    laserpub_.publish(laserpmap);

    if ( is_right )
      detepub_[0].publish( pointed_area );
    else
      detepub_[1].publish( pointed_area );

    return is_detected;

  }

  int  predict()
  {
   
     
    double predict_label;
    int kq;
   if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC))
     {
			predict_label = svm_predict_probability(model,x.ft,prob_estimates);
			/*	fprintf(output,"%g",predict_label);
			for(j=0;j<nr_class;j++)
				fprintf(output," %g",prob_estimates[j]);
			fprintf(output,"\n");

			*/

			cout << predict_label ;
				if ( predict_label == 1 )
				  {
				    //	    cout << "THIS IS YUBISHASHI !!!!! Prob:  " << prob_estimates[1] << endl << endl;
				    kq = 1;
				  }
			else 
			  {
			    //	    cout << "no no no no , Prob: " << prob_estimates[1] << endl << endl;
			    kq = 0;
			  }
		     
		}
    else
		{
			predict_label = svm_predict(model,x.ft);
			//	fprintf(output,"%g\n",predict_label);	fprintf(output,"%g\n",predict_label);

				cout << predict_label ;
					if ( predict_label == 1 )
				  {
				    //	    cout << " Yubisashi da !!!!!" << endl << endl;
				    kq = 1;
				  }
			else 
			  {
			    //   cout << " NO NO NO NO " << endl << endl;
			    kq = 0;
			  }
		}

   return kq;
  }

  void ProcessData( body_msgs::Skeletons skels, sensor_msgs::PointCloud2 cloud, sensor_msgs::PointCloud2 fullcloud)
  {
    if (skels.skeletons.size() == 0)
      return;
    count ++;
    int kq;
    double t1, fps;
    t1 = g_tock(t0); t0 = g_tick();
    fps = 1.0 / t1;
    //   cout << "time: " << t1 << endl;
    std::stringstream filename;
    pcl::PointCloud<pcl::PointXYZ> handcloud;
    // pcl::PointCloud<pcl::PointXYZ> fullcl;
    pcl::fromROSMsg( cloud, handcloud);
    
    
    arms a;
  
 

    extractArm( skels.skeletons[0], a);
    extractFeatures(handcloud, a, pointingmsg);
    kq = predict();

 
    laserpmap.polygons.clear();
    laserpmap.header = cloud.header;


     cv_bridge::CvImagePtr imgptr;

   
    imgptr = cv_bridge::toCvCopy( imgmsg, enc::BGR8);
    cv::Mat img = imgptr->image.clone();
   
    //send_data(img);
    float constant = 1.90476e-03;
    float centerX = 319.5;
    float centerY = 239.5;
    float radius = 0.1;
    
    int u,v,r;
    u = (int ) ( skels.skeletons[0].right_hand.position.x / constant / skels.skeletons[0].right_hand.position.z + centerX ); 
    v = (int ) ( skels.skeletons[0].right_hand.position.y / constant / skels.skeletons[0].right_hand.position.z + centerY ); 
    r = (int ) ( radius / constant / skels.skeletons[0].right_hand.position.z); 


    vector<Point3f> realpoints2;
    vector<Point2f> imagePoints2;

    
    pthread_mutex_lock(&mutex1);
    isPointing = 0;
    pthread_mutex_unlock(&mutex1);
    
    realpoints2.clear();
    imagePoints2.clear();

    realpoints2.push_back(cv::Point3f( - skels.skeletons[0].right_hand.position.x, skels.skeletons[0].right_hand.position.y, skels.skeletons[0].right_hand.position.z ));
    
    filename.str("");
    if ( kq == 1 )
      {
	int fingertip_x, fingertip_y;
	fingertip_x = (int ) ( pointingmsg.fingertip.x / constant / pointingmsg.fingertip.z + centerX );
	fingertip_y = (int ) ( pointingmsg.fingertip.y / constant / pointingmsg.fingertip.z + centerY );

	cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,255,0),2);
	if (predict_probability)
	  
	  filename << "YUBISASHI [ Prob: " << prob_estimates[0] << " ]";
	else
	  filename << "YUBISASHI"; 
	cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);


	pcl::PointXYZ min_pt, max_pt, huvo, pointPos, detected_center;
	if ( extractPointedArea( fullcloud, skels.skeletons[0], 1, min_pt, max_pt, huvo, pointPos, detected_center ) )
	   {
	     
	     if (started  && detected_num < 101)  
	       {
		 objectCloud.push_back(pointPos);
		 objectCloud2.push_back(detected_center);
		 detected_num ++;
		 
		 cout << "Added : " << detected_num << endl;
		 if (detected_num  == 100){
		   
		   // pcl::PointXYZ rHand;
		   //rHand = pointToPclPoint(
		   errorCompute(skels.skeletons[0].right_hand.position);
		   //		   pointingError.close();
		   key = 'q';
		 }
	       }
	     realpoints2.push_back(cv::Point3f(- min_pt.x, min_pt.y, min_pt.z));
	     realpoints2.push_back(cv::Point3f(- max_pt.x, max_pt.y, max_pt.z));


	     //isPointing = 1;
	     
	     	
	     pthread_mutex_lock(&mutex1);
	     isPointing = 1;
	     pthread_mutex_unlock(&mutex1);
	     int tmpx1, tmpx2, tmpy1, tmpy2;
	     tmpx1 = (int ) ( min_pt.x / constant / min_pt.z + centerX );
	     tmpy1 = (int ) ( min_pt.y / constant / min_pt.z + centerY );
	     tmpx2 = (int ) ( max_pt.x / constant / max_pt.z + centerX );
	     tmpy2 = (int ) ( max_pt.y / constant / max_pt.z + centerY );
	     
	     minx = (tmpx1 < tmpx2)?tmpx1:tmpx2;
	     maxx = (tmpx1 > tmpx2)?tmpx1:tmpx2;
	     miny = (tmpy1 < tmpy2)?tmpy1:tmpy2;
	     maxy = (tmpy1 > tmpy2)?tmpy1:tmpy2;
	   
	     cv::line( img, cv::Point( fingertip_x, fingertip_y ),  cv::Point(minx, miny),  cv::Scalar(0, 0, 255), 2 );
	     cv::line( img, cv::Point( fingertip_x, fingertip_y ),  cv::Point(maxx, maxy),  cv::Scalar(0, 0, 255), 2 );
	     cv::rectangle( img, cv::Point(minx, miny), cv::Point( maxx, maxy), cv::Scalar(255, 0, 0), 2);

	   }

	else
	  {

	    	
	pthread_mutex_lock(&mutex1);
	isPointing = 2;
	pthread_mutex_unlock(&mutex1);
	    int vocucx, vocucy;
	    
	    vocucx = (int ) ( huvo.x / constant / (abs(huvo.z)) + centerX );
	    vocucy = (int ) ( huvo.y / constant / (abs(huvo.z)) + centerY );
	    
	    realpoints2.push_back(cv::Point3f( - huvo.x, huvo.y, abs(huvo.z)));
	     cv::line( img, cv::Point( fingertip_x, fingertip_y ),  cv::Point(vocucx, vocucy),  cv::Scalar(0, 0, 255), 2 );
	  }

	pos++;
      }
    else
      {	
	cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,0,255),2);
	if (predict_probability)
	  
	  filename << "NON-YUBISASHI [ Prob: " << prob_estimates[1] << " ]" ;
	else
	  filename << "NON-YUBISASHI";
     
	cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
      }

    	projectPoints(Mat(realpoints2), rotation_vector, translation_vector, cameraMatrix, distCoeffs, imagePoints2);

	
	pthread_mutex_lock(&mutex1);
	handx = (int) imagePoints2[0].x;
	handy = (int) imagePoints2[0].y;
	
	if (isPointing == 1)
	  {
	    minx = (int) imagePoints2[1].x;
	    miny = (int) imagePoints2[1].y;
	    maxx = (int) imagePoints2[2].x;
	    maxy = (int) imagePoints2[2].y;
	    
	    
	   }
	else
	  if ( isPointing == 2){
	    minx = (int) imagePoints2[1].x;
	    miny = (int) imagePoints2[1].y;
	  }
	
	pthread_mutex_unlock(&mutex1);

    
    filename.str("");
    filename << "Frame: " << count;
    cv::putText( img, filename.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
    filename.str("");
    filename << "FPS: " << fps;
    cv::putText( img, filename.str(), cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
   
    cv::namedWindow("Hand Detect", 1);
    cv::imshow("Hand Detect", img);
    key = cv::waitKey(20);
   
    if ( key == 's'){
      //  cvDestroyWindow("Hand Detect");
      objectCloud.clear();
      objectCloud2.clear();
      //  getObjectPosition(img);
      kinectImg = img.clone();
      object_selected = false;
    
      cvSetMouseCallback("Hand Detect", &onMouseClick, this);
	
      while ( key != 'q' && (!object_selected) )
	{
	  cv::imshow("Hand Detect", kinectImg);
	  key = cv::waitKey(20);
	  // cout << "Key = " << key << endl;
	}
	
      cvDestroyWindow("Hand Detect");
  
    }
    else
        if ( key == 'c')
	  started = true;
    // If enable save result to image file
    if ( save != 0 ) 
      {
 
	filename.str("");
	filename << foldername << "/" << setfill('0') << setw(7) << count << "t" << save << kq  << ".pcd";
	pcl::io::savePCDFileASCII( filename.str().c_str(), handcloud);
    
	filename.str("");
	filename << foldername << "/" << setfill('0') << setw(7) << count << "t" << save << kq  <<".jpg";

	cv::imwrite(filename.str().c_str(), imgptr->image);
      }
  }

  /*\brief : Calculate the error between selected point and detected point
   *
   *
   *
   *
   *
   */
  void errorCompute(geometry_msgs::Point p)
  {
    cout << "Start Calculate" << endl; 

    Mat error1(1, 100, CV_64F);
    Mat error2(1, 100, CV_64F);
    

    // Calculate the Distance from user selected point to Detected Pointing center
    for (unsigned int i = 0; i < 100; i++ )
      {
	error1.at<double> (0, i) = sqrt((objectPosition.x - objectCloud[i].x)*(objectPosition.x - objectCloud[i].x) + 
					(objectPosition.y - objectCloud[i].y) * (objectPosition.y - objectCloud[i].y) + 	
					(objectPosition.z - objectCloud[i].z) * (objectPosition.z - objectCloud[i].z));

	error2.at<double> (0, i) = sqrt((objectPosition.x - objectCloud2[i].x)*(objectPosition.x - objectCloud2[i].x) + 
					(objectPosition.y - objectCloud2[i].y) * (objectPosition.y - objectCloud2[i].y) + 
					(objectPosition.z - objectCloud2[i].z) * (objectPosition.z - objectCloud2[i].z));
      }

    cout << "Mat created " << endl;
    Mat mean1, stddev1, mean2, stddev2;

    // Calculate Mean and Stddev of all 100 examples
    cv::meanStdDev(error1, mean1, stddev1);
    cv::meanStdDev(error2, mean2, stddev2);
    

    // Output to file
    //pointingError << "Object Position" << objectPosition << endl;
    //    pointingError << "Hand Position" << 
    //pointingError << "Pointing pos: Mean:  " <<  mean1.at<double> (0, 0) << "StdDev:  " <<  stddev1.at<double> (0, 0) << endl; 
    //pointingError << "Detected Center: Mean  " <<  mean2.at<double> (0, 0) << "StdDev:  " <<  stddev2.at<double> (0, 0) << endl;
    
    
    cout << "Mean Rows:" << mean1.rows << " Cols: " << mean1.cols << " " <<  "StdDev Rows:" << stddev1.rows << " Cols: " << stddev1.cols << endl; 
    pointingError << objectPosition.x << ", " << objectPosition.y << ", " << objectPosition.z <<", " << p.x << ", " << p.y << ", " << p.z << ", "  << mean1.at<double> (0, 0) << ", " << stddev1.at<double> (0, 0) << ", " << mean2.at<double> (0, 0) << ", " << stddev2.at<double> (0, 0) << endl; 
                 
    started = false;  
    detected_num = 0;
  }
  void getObjectPosition(cv::Mat& img)
  {
    kinectImg = img.clone();
    object_selected = false;
    cv::namedWindow("Select Object", 1);
    cvSetMouseCallback("Select Object", &onMouseClick, this);
	
    while ( key != 'q' && (!object_selected) )
      {
	cv::imshow("Select Object", img);
	key = cv::waitKey(20);
	cout << "Key = " << key << endl;
      }
	
    cvDestroyWindow("Select Object");
  }


  void fcloudcb( const sensor_msgs::PointCloud2ConstPtr &scan)
  {
    fcloudmsg = *scan;
    messageSync();

  }

  void cloudcb( const sensor_msgs::PointCloud2ConstPtr &scan )
  {

    pcloudmsg = *scan;
    messageSync();
    
  }
  
  void imgcb ( const sensor_msgs::ImageConstPtr & img)
  {
    pthread_mutex_lock(&mutex1);
    imgmsg = *img;
    pthread_mutex_unlock(&mutex1);
    //    send_data();
    messageSync();

  }
  void skelcb ( const body_msgs::SkeletonsConstPtr &skels)
  {
    skelmsg = *skels;

    messageSync();

  }

};

void* func_for_image(void *arg);
void* func_for_pointing(void *arg);

/****************
 *
 *
 *
 *
 *
 *            MAIN
 *
 *
 **
 */

int main( int argc, char ** argv )
{

  ros::init( argc, argv, "predict");
  ros::NodeHandle n;
  std::string name, folder;
  int save = 0, online = 1, prob;
  std::cout << "Please input the Hand MODEL filename: ";
  std::cin >> name;
  std::cout << "Using probability predict? : ";
  std::cin >> prob;
  
  // std::cout << "Would you like to save data? ( 0 for No, 1 for false positive, 2 for false nagative ): ";
  //std::cin >> save;
  if ( save != 0 ) 
    {
      std::cout << "Input folder name to save: ";
       std::cin >> folder;
    }
  HandSaver  saver(name, save, folder, online, prob);
  //pthread_t thread_for_image, thread_for_pointing;
  //pthread_create(&thread_for_image, NULL, &func_for_image, &saver);
  //  pthread_create(&thread_for_pointing, NULL, &func_for_pointing, &saver);
  ros::spin();

  return 0;
}








void* func_for_image(void *arg)
{
  HandSaver *saver = reinterpret_cast<HandSaver*>(arg);
  

      // Networking variables

  int sock, connected, istrue;
  struct sockaddr_in server_addr, client_addr;
  socklen_t sin_size;
  struct hostent *host;

    istrue = 1;  //   struct hostent *host;

  host = gethostbyname("133.19.23.174");
  server_addr.sin_family = AF_INET;         
  server_addr.sin_port = htons(5000);  
  server_addr.sin_addr = *((struct in_addr *)host->h_addr); 
    
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("Socket");
    exit(1);
  }

  if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&istrue,sizeof(int)) == -1) {
    perror("Setsockopt");
    exit(1);
  }
  //    server_addr.sin_addr.s_addr = INADDR_ANY;
  //inet_pton(AF_INET, "133.19.23.174", &server_addr.sin_addr.s_addr);
  bzero(&(server_addr.sin_zero),8); 

  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr))
      == -1) {
    perror("Unable to bind");
    exit(1);
  }

  if (listen(sock, 1) == -1) {
    perror("Listen");
    exit(1);
  }
		
  printf("\nImage TCPServer Waiting for client on port 5000");
  fflush(stdout);


  /* Variable for sending image */
   char send_buffer[1000000];
   char eob[] = {'h','n','a','m'};
   int bSize;
   int i = 0;
   double d = 1.2;	
   int image_size;
   int nRcv;
   char recvData[256];
   int counts;
  while(1) {
    /* listen & select */
    
     sin_size = sizeof(struct sockaddr_in);

     connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
     cout << "Image Server : ";

     if (connected < 1)
       {
	 cout << "Connected error !" << endl;
       }
     
     cout << " I got a connection from (" <<  inet_ntoa(client_addr.sin_addr) << ", " << ntohs(client_addr.sin_port) << " )" << endl;

    counts = 0;
    while(1) {
      /* send */

      //      cout << "Image Server : ";    
          
      cv_bridge::CvImagePtr imgptr;

   
      
      imgptr = cv_bridge::toCvCopy(saver->getImagemsg(), enc::BGR8);
      cv::Mat image = imgptr->image.clone();
     
      image_size = image.cols * image.rows * 3;

      bSize = image_size + sizeof(i)+sizeof(d);

      memcpy(send_buffer, image.data, image_size);
      memcpy(send_buffer + image_size, &i, sizeof(int));
      memcpy(send_buffer + image_size + sizeof(int), & d, sizeof(double));
      memcpy(send_buffer + bSize, eob, 4);
      
      if ( send(connected, send_buffer, bSize + 4, 0) < 0 ) 
	{

	    cout << "Stop sending data. Connection terminated " << endl;
	    close(connected);
	    break;
	  
	}
      else
	cv:: waitKey(10);
      /*      nRcv = recv( connected, recvData, 256, 0);
      if ( nRcv > 0 )
	{
	  recvData[nRcv] = '\0';
	  cout << "Recevied Signal : " << recvData << endl;
	  if( strcmp( recvData, "Quit" ) == 0 ){
	    cout << "Connection terminated. Stop sending data" << endl;
	    close(connected);
	    break;

	  }
	}
      */
      // cout << "Sent " << ++counts << " times" << endl;

    }
  }
  close(sock);
  return NULL;
}



void* func_for_pointing(void *arg)
{
  HandSaver *saver = reinterpret_cast<HandSaver*>(arg);
  

      // Networking variables

  int sock, connected, istrue;
  struct sockaddr_in server_addr, client_addr;
  socklen_t sin_size;
  struct hostent *host;

    istrue = 1;  //   struct hostent *host;

  host = gethostbyname("133.19.23.174");
  server_addr.sin_family = AF_INET;         
  server_addr.sin_port = htons(5001);  
  server_addr.sin_addr = *((struct in_addr *)host->h_addr); 
    
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("Socket");
    exit(1);
  }

  if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&istrue,sizeof(int)) == -1) {
    perror("Setsockopt");
    exit(1);
  }
  //    server_addr.sin_addr.s_addr = INADDR_ANY;
  //inet_pton(AF_INET, "133.19.23.174", &server_addr.sin_addr.s_addr);
  bzero(&(server_addr.sin_zero),8); 

  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr))
      == -1) {
    perror("Unable to bind");
    exit(1);
  }

  if (listen(sock, 1) == -1) {
    perror("Listen");
    exit(1);
  }
		
  printf("\nPointing Detection TCP Server Waiting for client on port 5001");
  fflush(stdout);


  /* Variable for sending image */
   char send_buffer[256];
  
   int bSize;
  
  
   int image_size;
   int nRcv;
   char recvData[24];
   int counts;


   int isPointing = saver->isPointing;
   int miny = saver->miny;
   int minx = saver->minx;
   int maxy = saver->maxy;
   int maxx = saver->maxx;
   int handx = saver->handx;
   int handy = saver->handy;
   
  while(1) {
    /* listen & select */
    
     sin_size = sizeof(struct sockaddr_in);

     connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);

     if (connected < 1)
       {
	 cout << "Connected error !" << endl;
       }

     cout << "Pointing Detection Server :" << endl;
     cout << " I got a connection from (" <<  inet_ntoa(client_addr.sin_addr) << ", " << ntohs(client_addr.sin_port) << " )" << endl;

    counts = 0;
    while(1) {
      /* send */
      
               
      // cout << "Pointing Detection Server : ";
       nRcv = recv( connected, recvData, 24, 0);
       if ( nRcv > 0 )
	{
	  recvData[nRcv] = '\0';
	  cout << "Pointing Detection Server : Recevied Signal : " << recvData << endl;
	   if( strcmp( recvData, "Quit" ) == 0 ){
	       cout << "Pointing Detection Server : Recevied TERMINAL Signal. Connection terminated " << endl;
	       close(connected);
	       break;
	      
	       }
	   
	   if( strcmp( recvData, "Get Finger" ) == 0 ){
      
	      pthread_mutex_lock(&mutex1);
	      isPointing = saver->isPointing;
	      miny  = saver->miny;
	      minx  = saver->minx;
	      maxy  = saver->maxy;
	      maxx  = saver->maxx;
	      handx = saver->handx;
	      handy = saver->handy;
	      pthread_mutex_unlock(&mutex1);
	      cout << "IsPointing :  " << isPointing << endl;
	      bSize = sizeof(int) * 7;

	      memcpy(send_buffer                  , &isPointing, sizeof(int));
	      memcpy(send_buffer + sizeof(int)    , &handx,      sizeof(int));
	      memcpy(send_buffer + sizeof(int) * 2, &handy,      sizeof(int));
	      memcpy(send_buffer + sizeof(int) * 3, &minx,       sizeof(int));
	      memcpy(send_buffer + sizeof(int) * 4, &miny,       sizeof(int));
	      memcpy(send_buffer + sizeof(int) * 5, &maxx,       sizeof(int));
	      memcpy(send_buffer + sizeof(int) * 6, &maxy,       sizeof(int));

	      
      
      
      
	      if ( send(connected, send_buffer, bSize , 0) < 0 ) 
		{

		  cout << "Pointing Detection Server : Stop sending data. Connection terminated " << endl;
		  close(connected);
		  break;
	  
		}
	      sleep(0.01);
	      	

	      cout << "Pointing Detection Server : Sent " << ++counts << " times" << endl;

	    
	      } // end if
	    
     	}
	    
    }
  }
  close(sock);
  return NULL;
}

void onMouseClick( int event, int x, int y, int flag,  void * param)
{
  HandSaver * data = (HandSaver *) param;
  switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
      
      //data->kinectPoints.push_back(cv::Point2f(x,y));
      pcl::PointXYZ p;
      if ( data->check_3d_position( x, y, p)){
	data->objectPosition = p;
	object_selected = true;
	cv::circle(data->kinectImg, cv::Point(x,y),2, cv::Scalar(0, 255, 0), 2);
      }
      else
	{
	cv::circle(data->kinectImg, cv::Point(x,y),2, cv::Scalar(0, 0, 255), 2);
	object_selected = false;
	}
      break;
      
    }
}
