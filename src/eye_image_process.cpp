#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <thread>

using namespace FlyCapture2;
using namespace std;
//using namespace ros;

int main(int argc, char **argv)
{
  if (argc != 2){
    ROS_INFO("usage : rosrun videostream eye_image_process camera , camera:left or right");
  }

  ros::init(argc, argv, "eye_image_process", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate;
  ros::Publisher pub_cnt;
  ros::Rate rate(60);
  ros::Timer timer;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub, pub_binary;
  sensor_msgs::ImagePtr msg, msg_binary;
  std_msgs::Float32 frameRate;
  std_msgs::UInt8 count;

  // variable setup
  int H_min, H_max, S_min, S_max, V_min, V_max;
  bool proc_minEnclosingCircle, proc_opening, proc_dilate;
  // vectors setup
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point> > contour;
  cv::Point2f center;
  float radius;

  unsigned int SerialNumber;

  string eye = argv[1];
  if (eye == "left"){
    SerialNumber = 17491073;
    pub = it.advertise("left_camera",1);
    pub_binary = it.advertise("left_camera_binary",1);
    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_left",1,false);
    pub_cnt = nh.advertise<std_msgs::UInt8>("count_left",1,false);
  }
  if (eye == "right"){
    SerialNumber = 17491067;
    pub = it.advertise("right_camera",1);
    pub_binary = it.advertise("right_camera_binary",1);
    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
    pub_cnt = nh.advertise<std_msgs::UInt8>("count_right",1,false);
  }

  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;

  // Connect camera
  busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
  error = camera.Connect(&guid);
  if (error != PGRERROR_OK){
    ROS_INFO("Failed to connect to right camera");
    return false;
  }

  error = camera.StartCapture();
  if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error != PGRERROR_OK){
    ROS_INFO("Failed to start image capture");
    return false;
  }

  cv::Mat img, img_binary, element;
  Image rawImage;
  Image bgrImage;
  unsigned int rowBytes;

  Property frmRate;
  frmRate.type = FRAME_RATE;
  error = camera.GetProperty(&frmRate);
  if (eye == "left"){
    cout << "Left camera setting frameRate = " << frmRate.absValue << endl;
    nh.getParam("/dynamic_HSV_server/H_min_L", H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L", H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L", S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L", S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L", V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L", V_max);
  }
  if (eye == "right"){
    cout << "Right camera setting frameRate = " << frmRate.absValue << endl;
    nh.getParam("/dynamic_HSV_server/H_min_R", H_min);
    nh.getParam("/dynamic_HSV_server/H_max_R", H_max);
    nh.getParam("/dynamic_HSV_server/S_min_R", S_min);
    nh.getParam("/dynamic_HSV_server/S_max_R", S_max);
    nh.getParam("/dynamic_HSV_server/V_min_R", V_min);
    nh.getParam("/dynamic_HSV_server/V_max_R", V_max);
  }

  double second;
  double fps;
  int cnt = 1;
  int morph_elem = 0;
  int morph_size = 1;

  double startt, endd;

  // Capture loop
  while (ros::ok()){
    //cout << "main thread on cpu:" << sched_getcpu() << endl;

    // Calculate right camera frame rate
    if (cnt == 1){
      startt = ros::Time::now().toSec();
    }
    /*
    if (eye == "left"){
      ROS_INFO("Left camera start capture image %d", cnt);
    }
    else {
      ROS_INFO("Right camera start capture image %d", cnt);
    }
    */
    error = camera.RetrieveBuffer(&rawImage);
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub.publish(msg);

    cv::cvtColor(img, img, CV_BGR2HSV);
    cv::inRange(img, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), img_binary);
/*
    // Open processing
    nh.getParam("/proc_opening", proc_opening);
    if (proc_opening == true){
      element = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
      cv::morphologyEx(img_binary, img_binary, 2, element);
    }
    // dilate processing
    nh.getParam("/proc_dilate", proc_dilate);
    if (proc_dilate == true){
      dilate(img_binary, img_binary, element);
    }

    cv::findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);

    for (int i = 0; i < contours.size(); i++){
      double area = cv::contourArea(contours[i]);
      if (area > 160){
        //cout << "area = " << area << endl;
        cv::minEnclosingCircle(contours[i], center, radius);
      }
    }
    contours.clear();
    hierarchy.clear();
*/
    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);

    ros::spinOnce();
    rate.sleep();
    cnt += 1;

    if (cnt == (int)frmRate.absValue){
      endd = ros::Time::now().toSec();
      second = endd - startt;
      fps = (double) frmRate.absValue / (second);
      frameRate.data = fps;
      pub_frameRate.publish(frameRate);

      //ROS_INFO("Taken time = %f", second);
      ROS_INFO("Estimate frame rate = %f \n", fps);
      //cout << /*"Taken time of right camera:" <<*/ second << endl;

      cnt = 1;
    }

  }

  error = camera.StopCapture();
  camera.Disconnect();

  return 0;
}
