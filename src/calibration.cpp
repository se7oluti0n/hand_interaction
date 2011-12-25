/*****
    Kinect and Network Camera Calibration
    

 */


#include <ros/ros.h>

// Msg
#include <body_msgs/Skeletons.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>

// Network

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
// OpenCV

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Thread

//#include <pthread.h>

// standart include
#include <stdio.h>
#include <stdlib.h>


using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

 
// Thread Mutex lock
//pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
char key;

void onMouseClick1( int event, int x, int y, int flag, void * param);
void onMouseClick2( int event, int x, int y, int flag, void * param);
void onMouseClickTest( int event, int x, int y, int flag, void * param);

class Calibration
{
private:
  ros::NodeHandle n_;
  ros::Subscriber cloudsub_, imgsub_;
  ros::Publisher cloudpub_;
  
  sensor_msgs::Image imgmsg, imgmsg2;
  sensor_msgs::PointCloud2 pcloudmsg;
  int lastimgseq;

  int sock;

  char totalBuff[300000];
  int nTotalsize;
  int lastnetworkseq, currentseq, lastcloudseq;


  Mat cameraMatrix, distCoeffs;
  vector<Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr;

  float aspectRatio;
  int flag;
public:
  cv::Mat kinectImg, ipImg;
  vector<Point2f> ipPoints, kinectPoints;
  vector<Point3f> realpoints;
 
  
  Calibration()
  {
    imgsub_ = n_.subscribe("/camera/rgb/image_color", 1, &Calibration::imgcb, this);
    
    cloudsub_ = n_.subscribe("/camera/rgb/points", 1, &Calibration::cloudcb, this);
    cloudpub_ = n_.advertise<sensor_msgs::PointCloud2> ("Current_Cloud", 1);
    initNetwork();
    
    lastimgseq = 0;
    imgmsg.header.seq = 0;


    lastnetworkseq = 0;
    currentseq = 0;
    
    lastcloudseq = 0;
    pcloudmsg.header.seq = 0;
    

    aspectRatio = 1.f;
    flag = 0;
    totalAvgErr = 0;
  }
  
  ~Calibration()
  {

  }
  

  void initNetwork()
  {
    
    

    char szHost[] = "133.19.22.239";
    char recv_data[32 * 1024];
    struct hostent *host;
    struct sockaddr_in server_addr; 
	

    host = gethostbyname(szHost);

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("Socket");
      exit(1);
    }

    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(80);   
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero),8); 

    if (connect(sock, (struct sockaddr *)&server_addr,
		sizeof(struct sockaddr)) == -1) 
      {
	perror("Connect");
	exit(1);
      }

    else cout << "Connect sucessfully" << endl;

    sprintf(recv_data,"GET /image?speed=30 HTTP/1.1\r\nHost: ");
    strcat(recv_data,szHost);
    strcat(recv_data,"\r\n\r\n");

    //データを送信（要求）
    send(sock,recv_data,(int)strlen(recv_data),0);
		
    //データを受信(1回目は画像データではない)
    recv(sock,recv_data,sizeof(recv_data),0);

  }

  void recvIPCamera()
  {
    int bytes_recieved;
    char recv_data[20000];
    
    nTotalsize = 0;
	  
    while ( ( bytes_recieved = recv(sock, recv_data, sizeof(recv_data),0))  > 0){

      if ( nTotalsize + bytes_recieved > 20000 )
	break;
      memcpy( totalBuff + nTotalsize, recv_data, bytes_recieved);
      nTotalsize += bytes_recieved;
      if ( ((int) recv_data[bytes_recieved - 3]) == '\n' &&  ((int)recv_data[bytes_recieved - 4] == '\r') &&
	   recv_data[bytes_recieved - 1] == '\n' &&  recv_data[bytes_recieved - 2] == '\r')
	break;
		
    }

    
    currentseq++;
   
  }

   void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
     pcloudmsg=*scan;
     //     cout << "Frame ID: " << pcloudmsg.header.frame_id  << endl;
     messageSync();
  }

  
  void imgcb ( const sensor_msgs::ImageConstPtr & img)
  {
    imgmsg = * img;
    recvIPCamera();
    messageSync();
  }
  


  int check_3d_position(int x, int y, pcl::PointXYZ &p)
  {
     pcl::PointCloud<pcl::PointXYZ> cloud;
     pcl::fromROSMsg( pcloudmsg, cloud);

     p = cloud.at( x, y);
     // p.x = - (p.x);
     
     cout << "2D: " << x << " " << y << "  >>  3D: " << p.x << " " << " " << p.y << " " << p.z << endl;
     if (p.z == NULL )
       return 0;
     else 
       return 1;
  }
  void messageSync()
  {
    if ( imgmsg.header.seq == lastimgseq || lastnetworkseq == currentseq || pcloudmsg.header.seq == lastcloudseq )
      return;
    
    double tdiff = ( imgmsg.header.stamp - pcloudmsg.header.stamp ).toSec();
    if (fabs(tdiff) < .15 ){
      lastimgseq = imgmsg.header.seq;
      lastnetworkseq = currentseq;
      ProcessData();
    }
  }

  void ProcessData()
  {
    cv_bridge::CvImagePtr imgptr = cv_bridge::toCvCopy(imgmsg, enc::BGR8);
    kinectImg = imgptr->image.clone();


  
    
    cloudpub_.publish(pcloudmsg);

    ipImg = cv::imdecode(cv::Mat(1, nTotalsize, CV_8UC1, totalBuff ), 1);
      
    
    //pthread_mutex_unlock(&mutex1);
     if (ipImg.data){
       
       cv::namedWindow("IP Camera", 1);
       cv::namedWindow("Kinect RGB", 1);
      
       cv::imshow("IP Camera", ipImg);
       cv::imshow("Kinect RGB", kinectImg);

       key  =  cv::waitKey(30);
       
       if (key == 's'){

	
	 ipPoints.clear();
	 kinectPoints.clear();
	 realpoints.clear();
	 cvSetMouseCallback("IP Camera", &onMouseClick1, this);
	 cvSetMouseCallback("Kinect RGB", &onMouseClick2, this);
      
	 cout << "Recieved " << nTotalsize  << " bytes" << endl;
	 while (key != 'q')
	   {

	      cv::imshow("IP Camera", ipImg);
	      cv::imshow("Kinect RGB", kinectImg);
	      key = cv::waitKey(20);
	   }

	 cvDestroyWindow("IP Camera");
	 cvDestroyWindow("Kinect RGB");

	 /*	 realpoints.push_back(Point3f(0.421148,  0.386885, 1.499));
	 //realpoints.push_back(Point3f( 0.47219,  0.393257, 1.48));
	 //realpoints.push_back(Point3f(0.546831,  0.404906, 1.461));
	 // realpoints.push_back(Point3f(0.597814,  0.41366, 1.443));
	 realpoints.push_back(Point3f(0.60102,  0.377511, 1.431));
	 //realpoints.push_back(Point3f(0.55062,  0.37398, 1.449));
	 //realpoints.push_back(Point3f(0.501574,  0.367449, 1.467));
	 //realpoints.push_back(Point3f(0.4458,  0.360886, 1.486));
	 //realpoints.push_back(Point3f(0.419454,  0.326866, 1.473));
	 //realpoints.push_back(Point3f(0.497471,  0.3395, 1.455));
	 //realpoints.push_back(Point3f(0.579214,  0.347529, 1.431));
	 realpoints.push_back(Point3f(0.604089,  0.328397, 1.419));
	 // realpoints.push_back(Point3f(0.521974,  0.309369, 1.431));
	 // realpoints.push_back(Point3f(0.47334,  0.30774, 1.449));
	 realpoints.push_back(Point3f(0.423334,  0.294797, 1.467));

	
	 ipPoints.push_back(Point2f(50, 107));
	 //ipPoints.push_back(Point2f(71, 93));
	 //ipPoints.push_back(Point2f(98, 77));
	 //ipPoints.push_back(Point2f(118, 65));
	 ipPoints.push_back(Point2f(120, 44));
	 //ipPoints.push_back(Point2f(102, 54));
	 //ipPoints.push_back(Point2f(81, 68));
	 //ipPoints.push_back(Point2f(62, 80));
	 //ipPoints.push_back(Point2f(51, 66));
	 //ipPoints.push_back(Point2f(80, 48));
	 //ipPoints.push_back(Point2f(109, 30));
	 ipPoints.push_back(Point2f(119, 5));
	 //ipPoints.push_back(Point2f(89, 21));
	 //ipPoints.push_back(Point2f(70, 34));
	 ipPoints.push_back(Point2f( 51, 41));
					 
	 */
	 //	 
	 //rvecs.resize(1);
	 //tvecs.resize(1);
	 rvecs.push_back(Mat::zeros(3,1,CV_64F));
	 tvecs.push_back(Mat::zeros(3,1,CV_64F));


	   cv::imwrite("corrKinect.jpg", kinectImg);
	   cv::imwrite("corrIP.jpg", ipImg);
	 runCalibration(cv::Size(320, 240), aspectRatio, flag, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, totalAvgErr, false);
	 for (int i = 1; i < 10; i++)

	   runCalibration(cv::Size(320, 240), aspectRatio, flag, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, totalAvgErr, true);


	        
       cv::namedWindow("Test IP Camera", 1);
       cv::namedWindow("Test Kinect RGB", 1);
      
       
       key = '#';
       while (key != 't'){
	 cv::imshow("Test IP Camera", ipImg);
	 cv::imshow("Test Kinect RGB", kinectImg);

	 key  =  cv::waitKey(30);
       }
       if (key == 't'){

	 cvSetMouseCallback("Test Kinect RGB", &onMouseClickTest, this);

	 while (key != 'q')
	   {
	     cv::imshow("Test IP Camera", ipImg);
	     cv::imshow("Test Kinect RGB", kinectImg);

	     cv::imwrite("testKinect.jpg", kinectImg);
	     cv::imwrite("testIP.jpg", ipImg);
	     key = cv::waitKey(20);
	   }

	 cvDestroyWindow("Test IP Camera");
	 cvDestroyWindow("Test Kinect RGB");

	 waitKey(0);
       }
       }

       

    }
    
     
    
     //cv::imshow("Kinect RGB", kinectImg);
    

  }

  /****************************************************************
   * \brief : Calibration processing
   *
   *
   *
   *
   *
   *****************************************************************/

 double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
    
    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);


        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }
    
    return std::sqrt(totalErr/totalPoints);
}


int  test_correspond(int x, int y, int &outx, int &outy)
  {
    vector<Point3f> realpoints2;
    vector<Point2f> imagePoints2;
    
    pcl::PointXYZ p;
    if ( check_3d_position( x, y, p)){
      realpoints2.push_back(cv::Point3f( p.x, p.y, p.z));
    
        
      
        projectPoints(Mat(realpoints2), rvecs[0], tvecs[0],
                      cameraMatrix, distCoeffs, imagePoints2);
	
	outx = (int) imagePoints2[0].x;
	outy = (int) imagePoints2[0].y;
	return 1;
    }
    else 
      return 0;
  }

  void load_camera_parameters( string input_file, Mat& cameraMatrix, Mat& distCoeffs)
  {
    FileStorage fs(input_file, FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    
  }
bool runCalibration(
                    Size imageSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs,vector< Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr,
		    bool useExtrinsicGuess = false)
  {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    //if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    //  cameraMatrix.at<double>(0,0) = aspectRatio;
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    
    vector<vector<Point3f> >  objectPoints(1);
    vector<vector<Point2f> > imagePoints(1);
    
    objectPoints[0] = realpoints;
    imagePoints[0] = ipPoints;

    //double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
    //			       distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS|CV_CALIB_RATIONAL_MODEL);
    
    load_camera_parameters( string("out_camera_data.yml"), cameraMatrix, distCoeffs );
    //Mat rvec, tvec;
 
    solvePnPRansac(Mat(realpoints),Mat( ipPoints), cameraMatrix, distCoeffs, rvecs[0], tvecs[0], useExtrinsicGuess);
 
    
 
  
    cout << "Camera Matrix: " <<cameraMatrix << endl;
    cout << "Distortion Coeffs: " << distCoeffs << endl;
    cout << cameraMatrix.depth() << " " << rvecs[0].depth() << endl;
    //  cout << "RMS error reported by calibrateCamera: " <<  rms << endl;
    
    //rvecs.push_back(rvec);
    //tvecs.push_back(tvec);
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
   

    cout << "Rotation Vector: " << rvecs[0] << endl;
    Mat rotationMatrix;
    Rodrigues(rvecs[0], rotationMatrix);
    cout << "Rotation Matrix: " << rotationMatrix << endl;
    cout << "Translation  vector:  " << tvecs[0] << endl;
  
    totalAvgErr = computeReprojectionErrors(objectPoints,imagePoints,
					    rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    cout << "Total Average Error: " << totalAvgErr << endl;
  


    return ok;
  }
  
};

//void *func_for_ipcamera(void *arg);
int main(int argc,char ** argv)
{
  ros::init(argc, argv, "Calibration");
  ros::NodeHandle n;
  Calibration cali;
  //  pthread_t thread_for_ip;
  //pthread_create(&thread_for_ip, NULL, &func_for_ipcamera, &cali);
  ros::spin();
  return 0;

}



void onMouseClick1( int event, int x, int y, int flag,  void * param)
{
  Calibration * data = (Calibration *) param;
  switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
      cv::circle(data->ipImg, cv::Point(x,y), 1, cv::Scalar(255, 0, 0), 2);
      data->ipPoints.push_back(cv::Point2f(x,y));
      cout << "Ip Camera :  " << x << " " << y << endl;
      break;
      
    }


}

void onMouseClick2( int event, int x, int y, int flag,  void * param)
{
  Calibration * data = (Calibration *) param;
  switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
      cv::circle(data->kinectImg, cv::Point(x,y),1, cv::Scalar(0, 0, 255), 2);
      data->kinectPoints.push_back(cv::Point2f(x,y));
      pcl::PointXYZ p;
      if ( data->check_3d_position( x, y, p))
	data->realpoints.push_back(cv::Point3f(  p.x, p.y, p.z));
      break;
      
    }

  

}



void onMouseClickTest( int event, int x, int y, int flag,  void * param)
{
    Calibration * data = (Calibration *) param;
  switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
      int outx, outy;
     
      cv::circle(data->kinectImg, cv::Point(x,y),1, cv::Scalar(0, 255, 0), 2);
      if (  data->test_correspond ( x, y, outx, outy) )
	cv::circle(data->ipImg, cv::Point(outx,outy),1, cv::Scalar(0, 255,0), 2);
      break;
      
    }
}
