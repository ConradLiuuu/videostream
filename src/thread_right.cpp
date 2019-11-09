#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <iostream>
#include <vector>

using namespace FlyCapture2;
using namespace std;
using namespace ros;

class Camera_
{
private:
  // flycapture setting
  unsigned int SerialNumber;
  unsigned int rowBytes;
  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;
  Image rawImage;
  Image bgrImage;
  Property frmRate;

  // openCV setting
  cv::Mat img, img_hsv, img_binary, img_ROI, element;
  int morph_elem;
  int morph_size;

  // vectors setup
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point> > contour;

  cv::Point2f center;
  float radius;

  // ros setting
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate, pub_binary_frameRate, pub_center, pub_rightDone;
  ros::Subscriber sub;
  std_msgs::Float32 frameRate, binary_frameRate;
  std_msgs::Float64MultiArray ball_center;
  std_msgs::Bool isDone;
  ros::Timer timer;

  // image transport setting
  sensor_msgs::ImagePtr msg_img, msg_binary, msg_ROI;
  image_transport::Publisher pub_img, pub_binary, pub_ROI;

  // variable setting
  unsigned int cnt, cnt_proc;
  double startt, endd, startt_proc, endd_proc;
  double second, fps, second_proc, fps_proc;

  int H_min, H_max, S_min, S_max, V_min, V_max;
  bool proc_minEnclosingCircle, proc_opening, proc_dilate;

public:
  Camera_(){
    SerialNumber = 17491067;

    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("right_camera",1);
    pub_binary = it.advertise("right_camera_binary",1);
    pub_ROI = it.advertise("right_camera_ROI",1);

    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
    pub_binary_frameRate = nh.advertise<std_msgs::Float32>("image_processing_frameRate_right",1,false);
    pub_center = nh.advertise<std_msgs::Float64MultiArray>("ball_center_right", 1, false);
    pub_rightDone = nh.advertise<std_msgs::Bool>("right_camera_done", 1, false);

    isDone.data = true;
    pub_rightDone.publish(isDone);

    cnt = 1;
    cnt_proc = 1;
    morph_elem = 0;
    morph_size = 1;

    center.x = 0;
    center.y = 0;
    radius = 0;

    nh.getParam("/dynamic_HSV_server/H_min_L", H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L", H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L", S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L", S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L", V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L", V_max);

    // connect camera
    busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    error = camera.Connect(&guid);

    frmRate.type = FRAME_RATE;
    error = camera.GetProperty(&frmRate);
    cout << "Right camera setting frameRate = " << frmRate.absValue << endl;

    // start capture image
    error = camera.StartCapture();
    timer = nh.createTimer(ros::Duration(1.0 / 100),std::bind(&Camera_::capture, this));
  }

  void capture(){
    if (cnt == 1){
      startt = ros::Time::now().toSec();
    }
    //ROS_INFO("cnt = %d", cnt);
    error = camera.RetrieveBuffer(&rawImage);
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);
/*
    if (center.x > 0 && center.y > 0 && (center.x+200)<2048 && (center.y+200)<1536){
      img_ROI = img(cv::Rect((int)center.x-200, (int)center.y-200, 400, 400));

      msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_ROI).toImageMsg();
      pub_ROI.publish(msg_ROI);
    }
*/
    msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub_img.publish(msg_img);
/*
    if (center.x > 0 && center.y > 0 && (center.x+200)<2048 && (center.y+200)<1536){
      //cout << center << endl;
      //if (img.rows > 500 && img.cols > 500){
      img_ROI = img(cv::Rect((int)center.x-200, (int)center.y-200, 400, 400));
      //cout << img_ROI.rows << " " << img_ROI.cols << endl;
      //}
      msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_ROI).toImageMsg();
      pub_ROI.publish(msg_ROI);
    }
*/
    cnt += 1;

    if (cnt == (int)frmRate.absValue){
      endd = ros::Time::now().toSec();
      second = endd - startt;
      fps = (double) frmRate.absValue / (second);
      frameRate.data = fps;
      //cout << fps << endl;
      pub_frameRate.publish(frameRate);

      cnt = 1;
    }
      
  }


  void operator()(){
    sub = nh.subscribe("/sampling_time", 1, &Camera_::callback, this);
  }

  void callback(const std_msgs::Bool::ConstPtr& msg){
    ROS_INFO("Right camera start to do image process %d", cnt_proc);

    startt_proc = ros::Time::now().toSec();
/*
    if (center.x > 0 && center.y > 0 && (center.x+200)<2048 && (center.y+200)<1536){
      //cout << center << endl;
      //if (img.rows > 500 && img.cols > 500){
      img_ROI = img(cv::Rect((int)center.x-200, (int)center.y-200, 400, 400));
      //cout << img_ROI.rows << " " << img_ROI.cols << endl;
      //}
      msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_ROI).toImageMsg();
      pub_ROI.publish(msg_ROI);
    }
*/
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
      //cout << "center = " << center.x << "," << center.y << endl;
      /*
      if (radius > 0){
        circle(img, center,radius,cv::Scalar(255,0,0), 3, 8,0);
      }
      */
      ball_center.data.push_back(center.x);
      ball_center.data.push_back(center.y);
      pub_center.publish(ball_center);
      contours.clear();
      hierarchy.clear();
      //radius = 0;
      ball_center.data.clear();
    }

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);

    cnt_proc += 1;

    endd_proc = ros::Time::now().toSec();
    second_proc = endd_proc - startt_proc;
    //cout << second_proc << endl;
    fps_proc = 1 / second_proc;
    binary_frameRate.data = fps_proc;
    pub_binary_frameRate.publish(binary_frameRate);

    //timer_isDone = nh.createTimer(ros::Duration(1.0 / 1),std::bind(&Camera_::isDone_func, this));
    /*
    for (int i = 0; i < 6; i++){
      pub_rightDone.publish(isDone);
      rate.sleep();
    }
    */

   //pub_rightDone.publish(isDone);
   //ros::spinOnce();
  }
/*
  void isDone_func(){
    pub_rightDone.publish(isDone);
  }*/
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_right");
  ros::NodeHandle nh;

  cout << "Right camera main thread on cpu:" << sched_getcpu() << endl;

  Camera_ camera;

  thread t1(ref(camera));


  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(5, &cpuset);
  int rc = pthread_setaffinity_np(t1.native_handle(), sizeof(cpu_set_t), &cpuset);
  //cout << "rc = " << rc << endl;
  if (rc != 0){
    std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
  }




  t1.join();

  ros::spin();

  return 0;
}
