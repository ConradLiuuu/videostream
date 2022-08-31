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

class CameraNode
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
  cv::Mat img, img_hsv, img_binary, img_ROI, element, img_serve;
  int morph_elem;
  int morph_size;

  // vectors setup
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point> > contour;

  cv::Point2f center;
  cv::Point2f center_last;
  cv::Point2f T_one2ori, T_two2one, delta;
  cv::Point2f center_in_world_frame;
  float radius;

  // ros setting
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate, pub_binary_frameRate, pub_center, pub_done;
  ros::Subscriber sub;
  std_msgs::Float32 frameRate, binary_frameRate;
  std_msgs::Float64MultiArray ball_center;
  std_msgs::Bool is_done;
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
  bool func;

  int img_x, img_y;

  std::string path;
  std::string fileName;
  std::string baseName;
  std::string num;
  vector<int> compression_params;

public:
  CameraNode(const std::string& name, const unsigned int& serial_number){
    SerialNumber = 17491073;
    baseName = name;

    path = "/home/lab606a/dic/" + baseName + "/";

    string tmp_string;

    image_transport::ImageTransport it(nh);

    tmp_string = baseName + "_camera";
    pub_img = it.advertise(tmp_string, 1);

    tmp_string = baseName + "_camera_binary";
    pub_binary = it.advertise(baseName, 1);

    tmp_string = baseName + "_amera_ROI";
    pub_ROI = it.advertise(baseName, 1);

    tmp_string = "frameRate_" + baseName;
    pub_frameRate = nh.advertise<std_msgs::Float32>(tmp_string, 1, false);

    tmp_string = "image_processing_frameRate_" + baseName;
    pub_binary_frameRate = nh.advertise<std_msgs::Float32>(tmp_string, 1, false);

    tmp_string = "ball_center_" + baseName;
    pub_center = nh.advertise<std_msgs::Float64MultiArray>(tmp_string, 1, false);

    tmp_string = baseName + "_camera_done";
    pub_done = nh.advertise<std_msgs::Bool>(tmp_string, 1, false);

    is_done.data = true;

    pub_done.publish(is_done);

    cnt = 1;
    cnt_proc = 1;
    morph_elem = 0;
    morph_size = 1;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);

    //center.x = 0;
    //center.y = 0;
    center = cv::Point2f(0,0);
    radius = 5;

    center_last = cv::Point2f(100,100);
    T_one2ori = cv::Point2f(650,230);

    nh.getParam("/dynamic_HSV_server/H_min_" + baseName, H_min);
    nh.getParam("/dynamic_HSV_server/H_max_" + baseName, H_max);
    nh.getParam("/dynamic_HSV_server/S_min_" + baseName, S_min);
    nh.getParam("/dynamic_HSV_server/S_max_" + baseName, S_max);
    nh.getParam("/dynamic_HSV_server/V_min_" + baseName, V_min);
    nh.getParam("/dynamic_HSV_server/V_max_" + baseName, V_max);

    // connect camera
    busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    error = camera.Connect(&guid);

    frmRate.type = FRAME_RATE;
    error = camera.GetProperty(&frmRate);
    cout << baseName << " camera setting frameRate = " << frmRate.absValue << endl;

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
    func = true;
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
    //func = false;

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
    ROS_INFO_STREAM(baseName << " camera start to do image process " << cnt_proc);

    num = std::to_string(ros::Time::now().toSec());
    fileName = path + baseName + std::to_string(cnt_proc) + "_" + num + ".jpg";
    cv::imwrite(fileName, img, compression_params);

    startt_proc = ros::Time::now().toSec();

    if ((center.x == 0) && (center.y == 0)){
      //cout << "not find ball \n";
      img_serve = img(cv::Rect(T_one2ori.x, T_one2ori.y, 640, 240));
      delta = cv::Point2f(0,0);
      center_in_world_frame = cv::Point2f(-1,-1);
      ball_center.data.push_back(center_in_world_frame.x);
      ball_center.data.push_back(center_in_world_frame.y);
      pub_center.publish(ball_center);
      ball_center.data.clear();
      //delta.x = 0;
      //delta.y = 0;
      //center_last.x = 200;
      //center_last.y = 200;
      //cout << img_serve.cols << ", " << img_serve.rows << endl;
    }
    else {
      if ((center_in_world_frame.x >=0) && (center_in_world_frame.x < 1848) && (center_in_world_frame.y >= 0) && (center_in_world_frame.y < 1336)){
        if (img_serve.cols == 640){
          T_two2one = center - center_last;

          center_in_world_frame = center_last + T_two2one + T_one2ori;

          img_x = (int)center_in_world_frame.x - 100;
          img_y = (int)center_in_world_frame.y - 100;
        }
        if (img_serve.cols == 200){
          delta = delta + (center - center_last);
          center_in_world_frame = center_last + T_two2one + T_one2ori + delta;
          img_x = (int)center_in_world_frame.x - 100;
          img_y = (int)center_in_world_frame.y - 100;
        }
        ball_center.data.push_back(center_in_world_frame.x);
        ball_center.data.push_back(center_in_world_frame.y);
        pub_center.publish(ball_center);
        img_serve = img(cv::Rect(img_x, img_y, 200, 200));
        ball_center.data.clear();
      }
      else {
        img_serve = img(cv::Rect(T_one2ori.x, T_one2ori.y, 640, 240));
        delta = cv::Point2f(0,0);
        center = cv::Point2f(0,0);
        center_in_world_frame = cv::Point2f(0,0);
      }
    }

    msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_serve).toImageMsg();
    pub_ROI.publish(msg_ROI);
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
    cv::cvtColor(img_serve, img_hsv, CV_BGR2HSV);
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
        if ((area > 160) && (radius >= 5) && (radius < 40)){
        //cout << baseName << " area = " << area << endl;
          cv::minEnclosingCircle(contours[i], center, radius);
        }
        //cout << baseName << " radius = " << radius << endl;
      }
      //cout << "center = " << center.x << "," << center.y << endl;
      /*
      if (radius > 0){
        circle(img, center,radius,cv::Scalar(255,0,0), 3, 8,0);
      }
      */
      //ball_center.data.push_back(center.x);
      //ball_center.data.push_back(center.y);
      //pub_center.publish(ball_center);
      contours.clear();
      hierarchy.clear();
      //radius = 0;
      //ball_center.data.clear();
    }

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);
  
    cnt_proc += 1;

    endd_proc = ros::Time::now().toSec();
    second_proc = endd_proc - startt_proc;
    //cout << second_proc << endl;
    /*
    fps_proc = 1 / second_proc;
    binary_frameRate.data = fps_proc;
    pub_binary_frameRate.publish(binary_frameRate);
    */
    //timer_isDone = nh.createTimer(ros::Duration(1.0 / 1),std::bind(&Camera_::isDone_func, this));
/*
    for(int i = 0; i < 6; i++){
      pub_done.publish(is_done);
      rate.sleep();
    }*/

    //pub_done.publish(is_done);
    //ros::spinOnce();
  }
/*
  void isDone_func(){
    pub_done.publish(is_done);
  }
*/
};
