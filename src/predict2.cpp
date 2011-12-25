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
#include <std_msgs/Bool.h>
#include <pcl_tools/pcl_utils.h>
#include <hand_interaction/Pointing.h>
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

#include <cv_bridge/cv_bridge.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include "svm.h"
using namespace std;
namespace enc = sensor_msgs::image_encodings;
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

vector<pcl::ModelCoefficients> coeffs;
//vector<vtkSmartPointer< vtkPolyData >> circles; 
boost::mt19937 gen;
pcl::visualization::CloudViewer viewer("Cloud Viewer");
pcl::PointCloud<pcl::PointXYZ> cylinder_cloud;
pcl::PointXYZ ff;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Features Visual Viewer"));

template <typename Point1, typename Point2>
void PointConversion(Point1 pt1, Point2 &pt2){
   pt2.x=pt1.x;
   pt2.y=pt1.y;
   pt2.z=pt1.z;
}

pcl::PointXYZ eigenToPclPoint(const Eigen::Vector4f &v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
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
  ros::Subscriber cloudsub_, skelsub_, imgsub_;
  ros::Publisher pointingpub_, fingerpub_;
  sensor_msgs::PointCloud2 pcloudmsg;
  body_msgs::Skeletons skelmsg;
  sensor_msgs::Image imgmsg;

  geometry_msgs::Point finger_point;
  
  std::vector<struct feature> frame;
  struct feature x;
  int max_nr_attr;
  
  struct svm_model *model;
  int predict_probability;
  
  stringstream filename;
  std::string name, foldername;
  int count, pos, neg ;
  int save;
  int lastskelseq, lastcloudseq, lastimgseq;
  timeval t0;

  int svm_type, nr_class;
  double *prob_estimates;
  int online;

  
  
public:
  
  HandSaver(std::string name_, int saveChoice, string folder, int onl = 1, int prob =  1):foldername(folder), name(name_), predict_probability(prob), max_nr_attr(64), save(saveChoice), online(onl)
    {
      if (online)
	{
	  cloudsub_ = n_.subscribe("hand1_fullcloud", 1, &HandSaver::cloudcb, this);      
	  skelsub_ = n_.subscribe("/skeletons", 1, &HandSaver::skelcb, this);
	  imgsub_  = n_.subscribe("/camera/rgb/image_color", 1, &HandSaver::imgcb, this);
	  // handpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("transformed_handcloud", 1);
	  pointingpub_ = n_.advertise<hand_interaction::Pointing> ("pointing_info", 1);
	  //fingerpub_ = n_.advertise<geometry_msgs::Point> ("fingertip", 1);
	}
      count = 0;
      pos = 0;
      neg = 0;
      t0 = g_tick();
      lastskelseq = 0;
      lastcloudseq = 0;
      lastimgseq = 0;
      skelmsg.header.seq = 0;
      pcloudmsg.header.seq = 0;
      imgmsg.header.seq = 0;
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
     
  
    if ( !online ) Process();
    }

  ~HandSaver()
  {
    // free(x);
    svm_free_and_destroy_model(&model);
    if ( predict_probability )
    free(prob_estimates);
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
    if ( skelmsg.header.seq == lastskelseq || pcloudmsg.header.seq == lastcloudseq || imgmsg.header.seq == lastimgseq)
      return;

    double tdiff = (skelmsg.header.stamp - pcloudmsg.header.stamp).toSec();
    double tdiff2 =  (imgmsg.header.stamp - pcloudmsg.header.stamp).toSec();
    double tdiff3 =  (imgmsg.header.stamp - skelmsg.header.stamp).toSec();
    
    if (fabs(tdiff) < .15 && fabs(tdiff2) < .15 && fabs(tdiff3) < .15 ){
      lastskelseq = skelmsg.header.seq;
      lastcloudseq = pcloudmsg.header.seq;
      lastimgseq = imgmsg.header.seq;
      ProcessData(skelmsg, pcloudmsg);
     }


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

   
   //    float r = ( max_pt.y - min_pt.y )* ( max_pt.y - min_pt.y) +  ( min_pt.x - max_pt.x) * ( min_pt.x - max_pt.x )  ;
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
    cout << "Max length : " << max_pt.z - min_pt.z << endl; 
    for ( int i = 0; i < 240; i++ )
      
      {
	histogram[i] = 0.0;
	//	cout << histogram[i];
      }

    /*********************************************

          
          Make model co


     *********************************************/
    // vector<pcl::ModelCoefficients> coeffs;
    /*  coeffs.clear();
    pcl::ModelCoefficients tmp;
    tmp.values.push_back(0.0);
    tmp.values.push_back(0.0);
    tmp.values.push_back(min_pt.z);

    tmp.values.push_back(0.0);
    tmp.values.push_back(0.0);
    tmp.values.push_back(1.0);

    tmp.values.push_back(r);
    
    coeffs.push_back(tmp);

    for (int i = 1; i < 7; i++ )
    { 
      (tmp.values[3]) += offset_z;
      coeffs.push_back(tmp);      
    }
    
    
    /*  circles.clear();
    pcl::ModelCoefficients tmp;
    tmp.values.push_back(0.0);
    tmp.values.push_back(0.0);
    tmp.values.push_back(0.4 * arm_length);

    float zzz = min_pt.z;
    for (int i = 0; i < 7 ; i++ )
      {
	
	vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (tmp, zzz);
	circles.push_back(data);
	zzz+= offset_z;
	}*/
    //  visual(output.makeShared(), coeffs);

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
    
     for ( int i = 0; i < output.points.size(); i++ )
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
				    cout << "THIS IS YUBISHASHI !!!!! Prob:  " << prob_estimates[1] << endl << endl;
				    kq = 1;
				  }
			else 
			  {
			    cout << "no no no no , Prob: " << prob_estimates[1] << endl << endl;
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
				    cout << " Yubisashi da !!!!!" << endl << endl;
				    kq = 1;
				  }
			else 
			  {
			    cout << " NO NO NO NO " << endl << endl;
			    kq = 0;
			  }
		}

   return kq;
  }
  void ProcessData( body_msgs::Skeletons skels, sensor_msgs::PointCloud2 cloud)
  {
    if (skels.skeletons.size() == 0)
      return;
    count ++;
    int kq;
    double t1, fps;
    t1 = g_tock(t0); t0 = g_tick();
    fps = 1.0 / t1;
    cout << "time: " << t1 << endl;
    std::stringstream filename;
    pcl::PointCloud<pcl::PointXYZ> handcloud;
    pcl::fromROSMsg( cloud, handcloud);
    arms a;
  
    hand_interaction::Pointing pointing;

    extractArm( skels.skeletons[0], a);
    extractFeatures(handcloud, a, pointing);
    kq = predict();

  
    //std_msgs::Bool is_yubisashi;
    if (kq == 1 ) pointing.is_pointing = true;
    else pointing.is_pointing = false;


    pointing.header = cloud.header;
    pointingpub_.publish(pointing);
    //fingerpub_.publish(finger_point);

     cv_bridge::CvImagePtr imgptr;

     // sensor_msgs::ImagePtr imgmsgptr(&imgmsg);
     imgptr = cv_bridge::toCvCopy( imgmsg, enc::BGR8);
    cv::Mat img = imgptr->image.clone();
    // imageHandDetect(img, );

    float constant = 1.90476e-03;
    float centerX = 319.5;
    float centerY = 239.5;
    float radius = 0.1;
    
    int u,v,r;
    u = (int ) ( skels.skeletons[0].right_hand.position.x / constant / skels.skeletons[0].right_hand.position.z + centerX ); 
    v = (int ) ( skels.skeletons[0].right_hand.position.y / constant / skels.skeletons[0].right_hand.position.z + centerY ); 
    r = (int ) ( radius / constant / skels.skeletons[0].right_hand.position.z); 
    
   

    //cv::imwrite(filename.str().c_str(), imgptr->image);
    filename.str("");
    if ( kq == 1 )
      {
	cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,255,0),2);
	if (predict_probability)
	  
	  filename << "YUBISASHI [ Prob: " << prob_estimates[0] << " ]";
	else
	  filename << "YUBISASHI"; 
	cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
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
    
    filename.str("");
    filename << "Frame: " << count;
    cv::putText( img, filename.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
    filename.str("");
    filename << "FPS: " << fps;
    cv::putText( img, filename.str(), cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
   
    //cv::imwrite(filename.str().c_str(), img);
    cv::imshow("Hand Detect", img);
    cv::waitKey(10);
    if ( save != 0 ) 
      {
 
    filename.str("");
    filename << foldername << "/" << setfill('0') << setw(7) << count << "t" << save << kq  << ".pcd";
    pcl::io::savePCDFileASCII( filename.str().c_str(), handcloud);
    
    filename.str("");
    filename << foldername << "/" << setfill('0') << setw(7) << count << "t" << save << kq  <<".jpg";
    
    // cv_bridge::CvImageConstPtr imgptr;

    //imgptr = cv_bridge::toCvShare( imgmsg, enc::BGR8);
    cv::imwrite(filename.str().c_str(), imgptr->image);
    /* filename.str("");
    filename << name << "/" << setfill('0') << setw(8) << count << ".txt";
    saveSkeletonsFileASCII( filename.str().c_str(), skels.skeletons[0]);
    std::cout << count << " "  <<  lastskelseq << " " << lastcloudseq << endl;/*/
    //  count++;
    
      }
  }

  void Process()
  {
    int classs;
    int kq;
    // cout << "Input class label( 1 / 0 ): " ;
    //cin >> classs;
  
  
    system("ls *pcd > PCDfileList");
    system("ls *txt > SkelFileList");
    system("ls *jpg > ImageFileList");
    system("mkdir converted");
    system("mkdir histogram");
  
    pcl::PointCloud<pcl::PointXYZ> handcloud, output;
    Eigen::Affine3f transformation;
   
    std::stringstream filename;
    //int count = 0;
    ifstream pcdin("PCDfileList");
    ifstream skelin("SkelFileList");
    ifstream imgin("ImageFileList"); 
    ofstream out("trainingdata.txt");
    while ( ! pcdin.eof() )
      {
	
	cout << "Start processing file " << count << endl;
	char fname[256];
	pcdin.getline( fname, 256 );
    
	if ( pcl::io::loadPCDFile( fname, handcloud ) == -1 )
	  {
	    // PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	    break;
	  }
   
	skelin.getline( fname, 256 );

	arms aa;
	readJointFile(fname, aa);
	
	hand_interaction::Pointing pointing;
	extractFeatures(handcloud, aa, pointing);
	 kq = predict();
	 


	 
	 count ++;
     
	 imgin.getline( fname, 256);

	 
	 cv::Mat img = cv::imread(fname);
	 // imageHandDetect(img, );

	 float constant = 1.90476e-03;
	 float centerX = 319.5;
	 float centerY = 239.5;
	 float radius = 0.1;
    
	 int u,v,r;
	 u = (int ) ( aa.right_hand.position.x / constant / aa.right_hand.position.z + centerX ); 
	 v = (int ) ( aa.right_hand.position.y / constant / aa.right_hand.position.z + centerY ); 
	 r = (int ) ( radius / constant / aa.right_hand.position.z); 
    
   

	 //cv::imwrite(filename.str().c_str(), imgptr->image);
	 filename.str("");
	 if ( kq == 1 )
	   {
	     cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,255,0),2);
	     if (predict_probability)
	       
	       filename << "YUBISASHI [ Prob: " << prob_estimates[1] << " ]";

	     else
	       
	       filename << "YUBISASHI" ; 
	     cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
	     pos++;
	   }
	 else
	   {	
	     cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,0,255),2);
	     if (predict_probability ) 
	       filename << "NON-YUBISASHI [ Prob: " << prob_estimates[1] << " ]" ;     
	     else
	       filename << "NON-YUBISASHI" ; 
	     cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);
	     neg++;
	   }
    
	 std::cout << "Sum : " << count << " Positive : " << pos << " Negative : " << neg << std::endl;
	 filename.str("");
	 filename << "Positive: " << pos << " / " <<  count;
	 cv::putText( img, filename.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
	 filename.str("");
   
	 
	 cv::imshow("Hand Detect", img);
	 while ( cv::waitKey(0) == -1 );
	 if ( save != 0 ) 
	   {
 
	     filename.str("");
	     filename << foldername << "/" << setfill('0') << setw(7) << count << "t" << save << kq  << ".pcd";
	     pcl::io::savePCDFileASCII( filename.str().c_str(), handcloud);
    
	     filename.str("");
	     filename << foldername << "/" << setfill('0') << setw(7) << count << "t" << save << kq  <<".jpg";
	     cv::imwrite(filename.str().c_str(), img);
    
	   }

      }
  }  
  void cloudcb( const sensor_msgs::PointCloud2ConstPtr &scan )
  {


    pcloudmsg = *scan;
    messageSync();
    
  }
  
  void imgcb ( const sensor_msgs::ImageConstPtr & img)
  {
    imgmsg = *img;
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
  std::cout << "Processing ONLINE? : ";
  std::cin >> online;
  HandSaver  saver(name, save, folder, online, prob);
  if ( online ) ros::spin();

  return 0;
}
