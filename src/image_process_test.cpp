#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <thread>

using namespace FlyCapture2;
using namespace std;
//using namespace ros;

void callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_process_left");
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_left",1,false);
  ros::Publisher pub_center = nh.advertise<std_msgs::Float64MultiArray>("ball_center_left", 1, false);
  ros::Subscriber sub;
  //ros::Publisher pub_cnt;
  ros::Rate rate(60);
  //ros::Timer timer;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("left_camera",1);
  image_transport::Publisher pub_ROI = it.advertise("left_camera_ROI",1);
  image_transport::Publisher pub_binary = it.advertise("left_camera_binary",1);
  sensor_msgs::ImagePtr msg, msg_binary;
  std_msgs::Float32 frameRate;
  std_msgs::Float64MultiArray ball_center;
  //std_msgs::UInt8 count;

  // variable setup
  int H_min, H_max, S_min, S_max, V_min, V_max;
  bool proc_minEnclosingCircle, proc_opening, proc_dilate;
  // vectors setup
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point> > contour;

  cv::Point2f center;
  float radius;
  center.x = 0;
  center.y = 0;
  radius = 0;

  unsigned int SerialNumber = 17491073;

  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;

  // Connect camera
  busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
  error = camera.Connect(&guid);
  if (error != PGRERROR_OK){
    ROS_INFO("Failed to connect to leftt camera");
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

  cv::Mat img, img_hsv, img_binary, img_ROI, element;
  Image rawImage;
  Image bgrImage;
  unsigned int rowBytes;

  Property frmRate;
  frmRate.type = FRAME_RATE;
  error = camera.GetProperty(&frmRate);
  ROS_INFO("Left camera setting frameRate = %f", frmRate.absValue);

  nh.getParam("/dynamic_HSV_server/H_min_L", H_min);
  nh.getParam("/dynamic_HSV_server/H_max_L", H_max);
  nh.getParam("/dynamic_HSV_server/S_min_L", S_min);
  nh.getParam("/dynamic_HSV_server/S_max_L", S_max);
  nh.getParam("/dynamic_HSV_server/V_min_L", V_min);
  nh.getParam("/dynamic_HSV_server/V_max_L", V_max);

  double second;
  double fps;
  int cnt = 1;
  int morph_elem = 0;
  int morph_size = 1;

  double startt, endd;
  //cv::namedWindow("left", 0);

  // Capture loop
  ROS_INFO("Start to do left camera image process");
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

    //sub = nh.subscribe("sampling_time",1,callback);
/*
    if (center.x > 0 && center.y > 0 && (center.x+200)<2048 && (center.y+200)<1536){
      //cout << center << endl;
      //if (img.rows > 500 && img.cols > 500){
      img_ROI = img(cv::Rect((int)center.x-200, (int)center.y-200, 400, 400));
      //cout << img_ROI.rows << " " << img_ROI.cols << endl;
      //}
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_ROI).toImageMsg();
      pub_ROI.publish(msg);
    }
*/
    ROS_INFO("Left camera start to do image process %d", cnt);

    cv::cvtColor(img, img_hsv, CV_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), img_binary);

    // Open processing
    nh.getParam("/dynamic_bool/proc_opening", proc_opening);
    if (proc_opening == true){
      element = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
      cv::morphologyEx(img_binary, img_binary, 2, element);
    }
    // dilate processing
    nh.getParam("/dynamic_bool/proc_dilate", proc_dilate);
    if (proc_dilate == true){
      dilate(img_binary, img_binary, element);
    }

    cv::findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    //pub_binary.publish(msg_binary);

    // minEnclosingCircle processing
    nh.getParam("/dynamic_bool/proc_minEnclosingCircle", proc_minEnclosingCircle);
    if (proc_minEnclosingCircle == true){
      for (int i = 0; i < contours.size(); i++){
        double area = cv::contourArea(contours[i]);
        if (area > 160){
        //cout << "area = " << area << endl;
          cv::minEnclosingCircle(contours[i], center, radius);
        }
      }
      //cout << "center = " << center.x << endl;
      if (radius > 0){
        circle(img, center,radius,cv::Scalar(255,0,0), 3, 8,0);
      }
      ball_center.data.push_back(center.x);
      ball_center.data.push_back(center.y);
      pub_center.publish(ball_center);
      contours.clear();
      hierarchy.clear();
      radius = 0;
      ball_center.data.clear();
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub.publish(msg);

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);

    //cv::imshow("left", img);
    //cv::waitKey(1);

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
      //ROS_INFO("Estimate frame rate = %f \n", fps);
      //cout << /*"Taken time of right camera:" <<*/ second << endl;

      cnt = 1;
    }

  }

  error = camera.StopCapture();
  camera.Disconnect();

  return 0;
}
