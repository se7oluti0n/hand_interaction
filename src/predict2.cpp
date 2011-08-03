#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
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



#include "svm.h"
using namespace std;
namespace enc = sensor_msgs::image_encodings;

struct svm_node *x;
int max_nr_attr = 64;

struct svm_model* model;
int predict_probability=0;

boost::mt19937 gen;


struct HandSaver
{
 private:
  ros::NodeHandle n_;
  ros::Subscriber cloudsub_, skelsub_, imgsub_;
  sensor_msgs::PointCloud2 pcloudmsg;
  body_msgs::Skeletons skelmsg;
  sensor_msgs::ImageConstPtr imgmsg;

  struct svm_node *x;
  int max_nr_attr;
  
  struct svm_model *model;
  int predict_probability;
  
  stringstream filename;
  std::string name, foldername;
  int count, pos, neg ;
  int save;
  int lastskelseq, lastcloudseq;
  timeval t0;

  int svm_type, nr_class;
  double *prob_estimates;

  
public:
  
  HandSaver(std::string name_, int saveChoice, string folder):foldername(folder), name(name_), predict_probability(1), max_nr_attr(64), save(saveChoice)
    {
      cloudsub_ = n_.subscribe("hand1_fullcloud", 1, &HandSaver::cloudcb, this);      
      skelsub_ = n_.subscribe("/skeletons", 1, &HandSaver::skelcb, this);
      imgsub_  = n_.subscribe("/camera/rgb/image_color", 1, &HandSaver::imgcb, this);
      count = 0;
      pos = 0;
      neg = 0;
      t0 = g_tick();
      lastskelseq = 0;
      lastcloudseq = 0;
      skelmsg.header.seq = 0;
      pcloudmsg.header.seq = 0;
      string cmd;
      cmd = "mkdir " + foldername;
      system(cmd.c_str());
      
      if ((model = svm_load_model(name.c_str())) == 0 )
      {
	cerr << "Can't open input file " << name << endl; 
	exit(1);
      }

     
      x = ( struct svm_node *) malloc ( 300 * sizeof(struct svm_node));

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
     
    }

  ~HandSaver()
  {
    free(x);
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
  void resample( pcl::PointCloud<pcl::PointXYZ> & cloudin, pcl::PointCloud<pcl::PointXYZ> & cloudout, body_msgs::Skeleton  &armin )
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

  void extractFeatures( pcl::PointCloud<pcl::PointXYZ> cloud, body_msgs::Skeleton skel )
  {
    pcl::PointCloud<pcl::PointXYZ> output, cloud2;
     EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
     EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
     Eigen::Matrix3f cov;
     Eigen::Vector4f centroid, direction,armvector;
  
     Eigen::Vector3f z_axis, y_axis, x_axis, origin;
     Eigen::Affine3f transformation;

     Eigen::Vector3f right_arm;

     resample ( cloud, cloud2, skel);
     right_arm[0] = skel.right_hand.position.x - skel.right_elbow.position.x;
     right_arm[1] = skel.right_hand.position.y - skel.right_elbow.position.y;
     right_arm[2] = skel.right_hand.position.z - skel.right_elbow.position.z;

     pcl::compute3DCentroid (cloud2, centroid);
     pcl::computeCovarianceMatrixNormalized(cloud2,centroid,cov);
     pcl::eigen33 (cov, eigen_vectors, eigen_values);

   
     z_axis[0] = eigen_vectors( 0, 2);
     z_axis[1] = eigen_vectors( 1, 2);
     z_axis[2] = eigen_vectors( 2, 2);
    
     /*  y_axis[0] = eigen_vectors( 0, 0);
	 y_axis[1] = eigen_vectors( 1, 0);
	 y_axis[2] = eigen_vectors( 2, 0);

	 origin [ 0 ] = centroid[0];
	 origin [ 1 ] = centroid[1];
	 origin [ 2 ] = centroid[2];
     */

     double cos = right_arm.dot(z_axis) / sqrt( right_arm.norm() * z_axis.norm() );
     if ( cos < 0 ) z_axis = - z_axis;
     // cout << " tan: "<< tann ;
    
     y_axis[0] = eigen_vectors( 0, 0);
     y_axis[1] = eigen_vectors( 1, 0);
     y_axis[2] = eigen_vectors( 2, 0);
  
     origin [ 0 ] = skel.right_hand.position.x;
     origin [ 1 ] = skel.right_hand.position.y;
     origin [ 2 ] = skel.right_hand.position.z;


     pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_axis, z_axis, origin, transformation);

     pcl::getTransformedPointCloud (cloud2, transformation, output);

     pcl::PointXYZ min_pt, max_pt;

     pcl::getMinMax3D( output, min_pt, max_pt);

   
     //    float r = ( max_pt.y - min_pt.y )* ( max_pt.y - min_pt.y) +  ( min_pt.x - max_pt.x) * ( min_pt.x - max_pt.x )  ;
     // r /= 4;

      
     float r1 = max_pt.y * max_pt.y + max_pt.x * max_pt.x  ;
     float r2 = min_pt.y * min_pt.y + min_pt.x * min_pt.x;
     float r3 = max_pt.y * max_pt.y + min_pt.x * min_pt.x;
     float r4 = min_pt.y * min_pt.y + max_pt.x * max_pt.x;

    
    
     float r = r1 > r2?r1:r2;
     r = r > r3?r:r3;
     r = r > r4?r:r4;

    
     float offset_r = r / 4.999;
     origin[2] = min_pt.z;
    
     float offset_z = (max_pt.z - min_pt.z) / 5.999;
     const double PI = 3.141592;
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
	 int pp = floor (( output.points[i].y * output.points[i].y  + output.points[i].x * output.points[i].x ) / offset_r );
	 int aa = floor (( PI + atan2(output.points[i].y, output.points[i].x)) / offset_a );

	 int index = zz * 40 + pp * 8 + aa % 8;
	 //if ( index < 0 || index > 239 ) cout << "Out of range!!" << " " << zz << " " << pp << " " << aa << endl;
	
	 histogram[ index  ] += 1.0;

       }
     
     for ( int i = 0; i < 240; i++ )
       {
	 histogram[i] /= (float) output.points.size();
	 x[i].index = i+1;
	 x[i].value = histogram[i];
	 // tcout << histogram[i] << endl;
       }
     x[240].index = -1;


  }


  int  predict()
  {
   
     
    double predict_label;
    int kq;
   if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC))
     {
			predict_label = svm_predict_probability(model,x,prob_estimates);
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
			predict_label = svm_predict(model,x);
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

    int kq;
    double t1, fps;
    t1 = g_tock(t0); t0 = g_tick();
    fps = 1.0 / t1;
    cout << "time: " << t1 << endl;
    std::stringstream filename;
    pcl::PointCloud<pcl::PointXYZ> handcloud;
    pcl::fromROSMsg( cloud, handcloud);
    extractFeatures(handcloud, skels.skeletons[0]);
    kq = predict();
    count ++;
     
     cv_bridge::CvImageConstPtr imgptr;

    imgptr = cv_bridge::toCvShare( imgmsg, enc::BGR8);
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
	filename << "YUBISASHI [ Prob: " << prob_estimates[1] << " ]";
	cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
	pos++;
      }
    else
      {	
	cv::circle( img, cv::Point(u,v), r, cv::Scalar(0,0,255),2);
	filename << "NOT YUBISASHI [ Prob: " << prob_estimates[1] << " ]" ;
     
	cv::putText( img, filename.str(), cv::Point(u,v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
      }
    
    filename.str("");
    filename << "Positive: " << pos << " / " <<  count;
    cv::putText( img, filename.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0));
    filename.str("");
    filename << "FPS: " << fps;
    cv::putText( img, filename.str(), cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0));
   
    //cv::imwrite(filename.str().c_str(), img);
    cv::imshow("Hand Detect", img);
    cv::waitKey(20);
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
    
  }
  
  void imgcb ( const sensor_msgs::ImageConstPtr & img)
  {
    imgmsg = img;
    // messageSync();

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
  int save;
  std::cout << "Please input the Hand MODEL filename: ";
  std::cin >> name;
  std::cout << "Would you like to save data? ( 0 for No, 1 for false positive, 2 for false nagative ): ";
  std::cin >> save;
  if ( save != 0 ) 
    {
      std::cout << "Input folder name to save: ";
       std::cin >> folder;
    }
  HandSaver  saver(name, save, folder);
  ros::spin();

  return 0;
}
