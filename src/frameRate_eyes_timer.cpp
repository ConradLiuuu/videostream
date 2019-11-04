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
//using namespace ros;

BusManager busMgr;
Error error;
Camera camera;
CameraInfo camInfo;
PGRGuid guid;

cv::Mat img;
Image rawImage;
Image bgrImage;
unsigned int rowBytes;

ros::Rate rate(60);
sensor_msgs::ImagePtr msg;
image_transport::Publisher pub;

int cnt = 1;

void callback(const ros::TimerEvent& event)
{
  camera.RetrieveBuffer(&rawImage);
  rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
  rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
  img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  pub.publish(msg);
  ros::spinOnce();
  rate.sleep();
  cnt += 1;
}

int main(int argc, char **argv)
{
  if (argc != 2){
    ROS_INFO("usage : rosrun videostream frameRate_eyes_timer camera , camera:left or right");
  }

  ros::init(argc, argv, "frameRate_eyes_timer", ros::init_options::AnonymousName);
  ros::Time::init();
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate;
  //ros::Rate ratce(30);
  ros::Timer timer;

  image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub;
  //sensor_msgs::ImagePtr msg;
  std_msgs::Float32 frameRate;

  unsigned int SerialNumber;

  string eye = argv[1];
  if (eye == "left"){
    SerialNumber = 17491073;
    pub = it.advertise("left_camera",1);
    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_left",1,false);
  }
  if (eye == "right"){
    SerialNumber = 17491067;
    pub = it.advertise("right_camera",1);
    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
  }
/*
  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;
*/
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

  // Capture loop
/*
  cv::Mat img;
  Image rawImage;
  Image bgrImage;
  unsigned int rowBytes;
*/
  Property frmRate;
  frmRate.type = FRAME_RATE;
  error = camera.GetProperty(&frmRate);
  if (eye == "left"){
    cout << "Left camera setting frameRate = " << frmRate.absValue << endl;
  }
  if (eye == "right"){
    cout << "Right camera setting frameRate = " << frmRate.absValue << endl;
  }

  double second;
  double fps;
  //int cnt = 1;

  clock_t start, end;

  while (ros::ok()){
    //cout << "main thread on cpu:" << sched_getcpu() << endl;

    // Calculate right camera frame rate
    if (cnt == 1){
      start = clock();
    }
    if (eye == "left"){
      ROS_INFO("Left camera start capture image %d", cnt);
    }
    else {
      ROS_INFO("Right camera start capture image %d", cnt);
    }
    timer = nh.createTimer(ros::Duration(1/121), callback);
/*
    error = camera.RetrieveBuffer(&rawImage);
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
    cnt += 1;
*/
    if (cnt == (int)frmRate.absValue){
      end = clock();
      second = (double) difftime(end, start) / CLOCKS_PER_SEC;
      fps = (double) frmRate.absValue / (second);
      frameRate.data = fps;
      pub_frameRate.publish(frameRate);

      //ROS_INFO("Taken time = %f", second);
      ROS_INFO("\n Estimate frame rate = %f \n", fps);
      //cout << /*"Taken time of right camera:" <<*/ second << endl;

      cnt = 1;
    }

  }

  error = camera.StopCapture();
  camera.Disconnect();

  return 0;
}

