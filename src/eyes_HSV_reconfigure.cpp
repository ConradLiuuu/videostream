#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "FlyCapture2.h"

using namespace FlyCapture2;
//using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_flycapture");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_img = it.advertise("origin",1);
  image_transport::Publisher pub_bin = it.advertise("binary",1);
  ros::Rate rate(10);
  sensor_msgs::ImagePtr msg_origin, msg_binary;
  int H_min, H_max,S_min, S_max,V_min, V_max;

  BusManager busMgr;

  Error error_L;
  Camera camera_L;
  CameraInfo camInfo_L;
  PGRGuid guid_L;

  Error error_R;
  Camera camera_R;
  CameraInfo camInfo_R;
  PGRGuid guid_R;

  // Connect camera
  busMgr.GetCameraFromIndex(1, &guid_L);
  error_L = camera_L.Connect(&guid_L);
  if (error_L != PGRERROR_OK){
    ROS_INFO("Failed to connect to camera");
    return false;
  }

  busMgr.GetCameraFromIndex(0, &guid_R);
  error_R = camera_R.Connect(&guid_R);
  if (error_R != PGRERROR_OK){
    ROS_INFO("Failed to connect to camera");
    return false;
  }

  // Get camera info
  error_L = camera_L.GetCameraInfo(&camInfo_L);
  if (error_L != PGRERROR_OK){
    ROS_INFO("Failed to get camera info");
    return false;
  }
  cout << "Vendor name_L :" << camInfo_L.vendorName << endl;
  cout << "Model name_L :" << camInfo_L.modelName << endl;
  cout << "Serial number_L :" << camInfo_L.serialNumber << endl;

  error_R = camera_R.GetCameraInfo(&camInfo_R);
  if (error_R != PGRERROR_OK){
    ROS_INFO("Failed to get camera info");
    return false;
  }
  cout << "Vendor name_R :" << camInfo_R.vendorName << endl;
  cout << "Model name_R :" << camInfo_R.modelName << endl;
  cout << "Serial number_R :" << camInfo_R.serialNumber << endl;

  // Start capture image
  error_L = camera_L.StartCapture();
  if ( error_L == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_L != PGRERROR_OK){
    ROS_INFO("Failed to start image capture");
    return false;
  }

  error_R = camera_R.StartCapture();
  if ( error_R == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_R != PGRERROR_OK){
    ROS_INFO("Failed to start image capture");
    return false;
  }

  // capture loop

  Image rawImage;
  Image bgrImage;
  Image rawImage_R;
  Image bgrImage_R;

  unsigned int rowBytes;
  unsigned int rowBytes_R;
  cv::Mat img, img_hsv;
  cv::Mat img_R, img_hsv_R;

  cv::Mat threshold = cv::Mat::zeros(1536, 2048, CV_8U);
  cv::Mat threshold_R = cv::Mat::zeros(1536, 2048, CV_8U);

  Error err;
  Error err_R;
  char key = 0;

  while (key != 'q'){
  //while (ros::ok()){
    // Get image
    //Image rawImage;
    Error err = camera_L.RetrieveBuffer(&rawImage);
    Error err_R = camera_R.RetrieveBuffer(&rawImage_R);

    if (err != PGRERROR_OK){
      ROS_INFO("Captrue error");
      continue;
    }
    
    // Convert to bgr
    //Image bgrImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
    rawImage_R.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_R);

    // Conver to OpenCV Mat
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);
    rowBytes_R = (double)bgrImage_R.GetReceivedDataSize() / (double)bgrImage_R.GetRows();
    img_R = cv::Mat(bgrImage_R.GetRows(), bgrImage_R.GetCols(), CV_8UC3, bgrImage_R.GetData(), rowBytes_R);
/*
    ROS_INFO("rows = %d", bgrImage.GetRows());
    ROS_INFO("cols = %d", bgrImage.GetCols());
*/
    //cv::resize(img, img, cv::Size(), 0.315, 0.315);
/*
    img_hsv = img.clone();
    cv::cvtColor(img_hsv, img_hsv, CV_BGR2HSV);

    nh.getParam("/dynamic_HSV_server/H_min_L",H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L",H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L",S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L",S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L",V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L",V_max);

    cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), threshold);

    msg_origin = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
    
    pub_img.publish(msg_origin);
    pub_bin.publish(msg_binary);
    ros::spinOnce();
    rate.sleep();
*/
    cv::namedWindow("image", 0);
    cv::imshow("image", img);
    cv::namedWindow("image R", 0);
    cv::imshow("image R", img_R);
    key = cv::waitKey(30);
  }

  //error = camera.StopCapture();
  camera_L.Disconnect();

  return 0;
}
