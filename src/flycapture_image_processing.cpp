#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <ctime>

using namespace FlyCapture2;
using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  // ros setup
  init(argc, argv, "flycapture_image_processing");
  NodeHandle nh;
  Rate rate(30);
  // iamge transport setup
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left = it.advertise("left_camera",1);
  image_transport::Publisher pub_left_binary = it.advertise("left_camera_binary",1);
  image_transport::Publisher pub_right = it.advertise("right_camera",1);
  image_transport::Publisher pub_right_binary = it.advertise("right_camera_binary",1);
  // message setup
  sensor_msgs::ImagePtr msg_left, msg_right;
  // variable setup
  int H_min_L, H_max_L, S_min_L, S_max_L, V_min_L, V_max_L;
  int H_min_R, H_max_R, S_min_R, S_max_R, V_min_R, V_max_R;
  bool proc_minEnclosingCircle, proc_opening, proc_dilate;
  // vectors setup
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point> > contour;
  cv::Point2f center;
  float radius;
  // set serial number of cameras
  unsigned int SerialNumberL = 17491073;
  unsigned int SerialNumberR = 17491067;

  BusManager busMgr;

  // Left eye 
  Error error_L;
  Camera camera_L;
  CameraInfo camInfo_L;
  PGRGuid guid_L;

  // Right eye
  Error error_R;
  Camera camera_R;
  CameraInfo camInfo_R;
  PGRGuid guid_R;

  // Connect camera
  busMgr.GetCameraFromSerialNumber(SerialNumberL, &guid_L);
  error_L = camera_L.Connect(&guid_L);
  if (error_L != PGRERROR_OK){
    //ROS_INFO("Failed to connect to left camera");
    return false;
  }

  busMgr.GetCameraFromSerialNumber(SerialNumberR, &guid_R);
  error_R = camera_R.Connect(&guid_R);
  if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to connect to right camera");
    return false;
  }

  // Start capture image
  error_L = camera_L.StartCapture();
  if ( error_L == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    //ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_L != PGRERROR_OK){
    //ROS_INFO("Failed to start image capture");
    return false;
  }

  error_R = camera_R.StartCapture();
  if ( error_R == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    //ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to start image capture");
    return false;
  }

  // OpenCV iamge setup
  cv::Mat img_L, img_R, img_L_binary, img_R_binary;
  //cv::namedWindow("image left", 0);
  //cv::namedWindow("image left binary", 0);
  //cv::namedWindow("image left after open", 0);
  //cv::namedWindow("image right", 0);
  //cv::namedWindow("image right binary", 0);

  // flycapture image setup
  Image rawImage_L;
  Image bgrImage_L;
  Image rawImage_R;
  Image bgrImage_R;

  unsigned int rowBytes_L;
  unsigned int rowBytes_R;

  Error err_L;
  Error err_R;

  char key = 0;
  int morph_elem = 0;
  int morph_size = 1;

  // Get HSV parameter
  nh.getParam("/dynamic_HSV_server/H_min_L", H_min_L);
  nh.getParam("/dynamic_HSV_server/H_max_L", H_max_L);
  nh.getParam("/dynamic_HSV_server/S_min_L", S_min_L);
  nh.getParam("/dynamic_HSV_server/S_max_L", S_max_L);
  nh.getParam("/dynamic_HSV_server/V_min_L", V_min_L);
  nh.getParam("/dynamic_HSV_server/V_max_L", V_max_L);

  nh.getParam("/dynamic_HSV_server/H_min_L", H_min_R);
  nh.getParam("/dynamic_HSV_server/H_max_L", H_max_R);
  nh.getParam("/dynamic_HSV_server/S_min_L", S_min_R);
  nh.getParam("/dynamic_HSV_server/S_max_L", S_max_R);
  nh.getParam("/dynamic_HSV_server/V_min_L", V_min_R);
  nh.getParam("/dynamic_HSV_server/V_max_L", V_max_R);

  // timer setup
  clock_t opening_start, opening_end;
  clock_t contour_timer_start, contour_timer_end, minC_start, minC_end;
  clock_t thre_start;
  double seconds;
  cv::Mat element;

  // capture loop
  while (ros::ok()){
    // get buffer data and write to flycapture iamge
    err_L = camera_L.RetrieveBuffer(&rawImage_L);
    // convert rawimage to BGR format
    rawImage_L.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_L);
    rowBytes_L = (double)bgrImage_L.GetReceivedDataSize() / (double)bgrImage_L.GetRows();
    // convert image format from flycapture format to OpenCV format
    img_L = cv::Mat(bgrImage_L.GetRows(), bgrImage_L.GetCols(), CV_8UC3, bgrImage_L.GetData(), rowBytes_L);

    // doing threshold
    thre_start = clock();
    cv::cvtColor(img_L, img_L, CV_BGR2HSV);
    cv::inRange(img_L, cv::Scalar(H_min_L, S_min_L, V_min_L), cv::Scalar(H_max_L, S_max_L, V_max_L), img_L_binary);

    opening_start = clock();
    //seconds = (double) difftime(opening_start, thre_start) / CLOCKS_PER_SEC;
    //cout << "Threshold processing takes " << seconds << " sec" << endl;

    // Open processing
    nh.getParam("/proc_opening", proc_opening);
    if (proc_opening == true){
      element = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
      cv::morphologyEx(img_L_binary, img_L_binary, 2, element);
      //cv::imshow("image left after open", img_L_binary);
    }
    opening_end = clock();
    seconds = (double) difftime(opening_end, opening_start) / CLOCKS_PER_SEC;
    //cout << seconds << endl;
    //cout << "Open process takes " << seconds << " sec" << endl;

    nh.getParam("/proc_dilate", proc_dilate);
    if (proc_dilate == true){
      dilate(img_L_binary, img_L_binary, element);
    }

    cv::findContours(img_L_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //cv::imshow("image left binary", img_L_binary);

    msg_left = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_L_binary).toImageMsg();
    pub_left_binary.publish(msg_left);
    ros::spinOnce();

    contour_timer_start = clock();
    
    for (int i = 0; i < contours.size(); i++){
      double area = cv::contourArea(contours[i]);
      if (area > 160){
        //cout << "area = " << area << endl;
        cv::minEnclosingCircle(contours[i], center, radius);
      }
    }

    contour_timer_end = clock();
    seconds = (double) difftime(contour_timer_end, contour_timer_start) / CLOCKS_PER_SEC;
    //cout << "Select contours takes " << seconds << " sec" << endl;
    // release vectors
    contours.clear();
    //contour.clear();
    hierarchy.clear();
    // update images
    key = cv::waitKey(1);
  }

  error_L = camera_L.StopCapture();
  camera_L.Disconnect();

  return 0;
}
