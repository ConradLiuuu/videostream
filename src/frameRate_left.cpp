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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "frameRate_left");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left = it.advertise("left_camera",1);
  //image_transport::Publisher pub_right = it.advertise("right_camera",1);
  ros::Rate rate(30);
  sensor_msgs::ImagePtr msg_left, msg_right;

  ros::Publisher pub_frameRate_left = nh.advertise<std_msgs::Float32>("frameRate_left",1,false);
  //ros::Publisher pub_frameRate_right = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
  std_msgs::Float32 frameRate_left;


  unsigned int SerialNumberL = 17491073;
  //unsigned int SerialNumberR = 17491067;

  BusManager busMgr;

  // Left eye 
  Error error_L;
  Camera camera_L;
  CameraInfo camInfo_L;
  PGRGuid guid_L;
/*
  // Right eye
  Error error_R;
  Camera camera_R;
  CameraInfo camInfo_R;
  PGRGuid guid_R;
*/
  // Connect camera
  busMgr.GetCameraFromSerialNumber(SerialNumberL, &guid_L);
  error_L = camera_L.Connect(&guid_L);
  if (error_L != PGRERROR_OK){
    //ROS_INFO("Failed to connect to left camera");
    return false;
  }
/*
  busMgr.GetCameraFromSerialNumber(SerialNumberR, &guid_R);
  error_R = camera_R.Connect(&guid_R);
  if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to connect to right camera");
    return false;
  }
*/
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
/*
  error_R = camera_R.StartCapture();
  if ( error_R == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    //ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to start image capture");
    return false;
  }
*/
  // capture loop
  cv::Mat img_L, img_R;

  Image rawImage_L;
  Image bgrImage_L;
  Image rawImage_R;
  Image bgrImage_R;

  unsigned int rowBytes_L;
  unsigned int rowBytes_R;

  Error err_L;
  Error err_R;

  char key = 0;

  //cv::namedWindow("image left", 0);
  //cv::namedWindow("image right", 0);

  Property frmRate_L;
  frmRate_L.type = FRAME_RATE;
  err_L = camera_L.GetProperty(&frmRate_L);
  cout << "Left camera setting frameRate = " << frmRate_L.absValue << endl;
/*
  Property frmRate_R;
  frmRate_R.type = FRAME_RATE;
  err_R = camera_R.GetProperty(&frmRate_R);
  cout << "Right camera setting frameRate = " << frmRate_R.absValue << endl;
*/
  int num_frames_L = frmRate_L.absValue;
  //int num_frames_R = frmRate_R.absValue;
  double second;
  double fps;

  clock_t start_L, end_L, start_R, end_R;

  //while (key != 'q'){
  while (ros::ok()){
    //cout << "main thread on cpu:" << sched_getcpu() << endl;
    // Calculate left camera frame rate
    start_L = clock();
    for (int i = 0; i  < num_frames_L; i++){
      err_L = camera_L.RetrieveBuffer(&rawImage_L);
      rawImage_L.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_L);
      rowBytes_L = (double)bgrImage_L.GetReceivedDataSize() / (double)bgrImage_L.GetRows();
      img_L = cv::Mat(bgrImage_L.GetRows(), bgrImage_L.GetCols(), CV_8UC3, bgrImage_L.GetData(), rowBytes_L);
      //cv::imshow("image left", img_L);
      //key = cv::waitKey(1);
      msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_L).toImageMsg();
      pub_left.publish(msg_left);
      ros::spinOnce();
      rate.sleep();
    }
    end_L = clock();
    second = (double) difftime(end_L, start_L) / CLOCKS_PER_SEC;
    fps = (double) num_frames_L / (second); 
    frameRate_left.data = fps;
    pub_frameRate_left.publish(frameRate_left);
    ROS_INFO("Taken time = %f", second);
    ROS_INFO("Estimate frame rate = %f", fps);
    //ros::spinOnce();

    
    //cout << "Taken time of left camera:" << second << endl;
    //cout << "Estimated left camera frame rate:" << fps << endl << endl;
/*
    // Calculate right camera frame rate
    start_R = clock();
    for (int i = 0; i  < num_frames_R; i++){
      err_R = camera_R.RetrieveBuffer(&rawImage_R);
      rawImage_R.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_R);
      rowBytes_R = (double)bgrImage_R.GetReceivedDataSize() / (double)bgrImage_R.GetRows();
      img_R = cv::Mat(bgrImage_R.GetRows(), bgrImage_R.GetCols(), CV_8UC3, bgrImage_R.GetData(), rowBytes_R);
      //cv::imshow("image right", img_R);
      //key = cv::waitKey(1);
      msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_R).toImageMsg();
      pub_right.publish(msg_right);
      ros::spinOnce();
    }
    end_R = clock();
    second = (double) difftime(end_R, start_R) / CLOCKS_PER_SEC;
    fps = (double) num_frames_R / (second);
    frameRate_right.data = fps;
    pub_frameRate_right.publish(frameRate_right);
    //ros::spinOnce();
    cout << "Taken time of right camera:" << second << endl;
    cout << "Estimated right camera frame rate:" << fps << endl << endl;
*/
    //key = cv::waitKey(1); // wait 1 milli second
    //rate.sleep();

  }

  error_L = camera_L.StopCapture();
  camera_L.Disconnect();

  return 0;
}
