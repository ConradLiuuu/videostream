#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64MultiArray.h>
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
  cv::Mat img, img_hsv, img_binary, img_ROI, element, img_serve, img2;
  int morph_elem;
  int morph_size;

  // vectors setup
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point> > contour;

  cv::Point2f center;
  cv::Point2i center_int_type;
  cv::Point2i center_last;
  cv::Point2i T_one2ori, T_two2one, delta;
  cv::Point2i center_in_world_frame;
  float radius;

  // ros setting
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate, pub_binary_frameRate, pub_center, pub_leftDone;
  ros::Subscriber sub;
  std_msgs::Float32 frameRate, binary_frameRate;
  std_msgs::Int64MultiArray ball_center;
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
  bool func;

  int img_x, img_y;

public:
  std::string path = "/home/lab606a/dic/left/";
  std::string fileName_L;
  std::string baseName_L = "left";
  std::string num;
  vector<int> compression_params;

  Camera_(){
    SerialNumber = 17491073;

    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("left_camera",1);
    pub_binary = it.advertise("left_camera_binary",1);
    pub_ROI = it.advertise("left_camera_ROI",1);

    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_left",1,false);
    pub_binary_frameRate = nh.advertise<std_msgs::Float32>("image_processing_frameRate_left",1,false);
    pub_center = nh.advertise<std_msgs::Int64MultiArray>("ball_center_left", 1, false);
    pub_leftDone = nh.advertise<std_msgs::Bool>("left_camera_done", 1, false);

    isDone.data = true;

    pub_leftDone.publish(isDone);

    cnt = 1;
    cnt_proc = 1;
    morph_elem = 0;
    morph_size = 1;
    //compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    //center.x = 0;
    //center.y = 0;
    center = cv::Point2f(0,0);
    radius = 5;

    center_int_type = cv::Point2i(0,0);
    center_last = cv::Point2i(100,100);
    T_one2ori = cv::Point2i(670,40);

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
    cout << "Left camera setting frameRate = " << frmRate.absValue << endl;

    // start capture image
    error = camera.StartCapture();
    timer = nh.createTimer(ros::Duration(1.0 / 121),std::bind(&Camera_::capture, this));
    //sub = nh.subscribe("/sampling_time", 1, &Camera_::callback, this);
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
    //ROS_INFO("left cnt = %d", cnt);
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
    //img2 = img.clone();
    ROS_INFO("Left camera start to do image process %d", cnt_proc);
/*
    num = std::to_string(ros::Time::now().toSec());
    fileName_L = path + baseName_L + std::to_string(cnt_proc) + "_" + num + ".jpg";
    cv::imwrite(fileName_L, img2, compression_params);
*/

    startt_proc = ros::Time::now().toSec();

    if ((center_int_type.x == 0) && (center_int_type.y == 0)){
      //cout << "not find ball \n";
      img_serve = img(cv::Rect(T_one2ori.x, T_one2ori.y, 640, 240));
      delta = cv::Point2i(0,0);
      center_in_world_frame = cv::Point2i(-1,-1);
      ball_center.data.push_back(cnt_proc);
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
      if ((center_int_type.x >0) && (center_in_world_frame.x < 1848) && (center_int_type.y > 0) && (center_in_world_frame.y < 1336)){
      //if ((center_in_world_frame.x >=0) && (center_in_world_frame.x < 1848) && (center_in_world_frame.y >= 0) && (center_in_world_frame.y < 1336)){
        if (img_serve.cols == 640){
          T_two2one = center_int_type - center_last;

          center_in_world_frame = center_last + T_two2one + T_one2ori;
          cout << "left center640 = " << center_in_world_frame << endl;

          img_x = center_in_world_frame.x - 100;
          img_y = center_in_world_frame.y - 100;
        }
        if (img_serve.cols == 200){
          delta = delta + (center_int_type - center_last);
          center_in_world_frame = center_last + T_two2one + T_one2ori + delta;
          cout << "left center = " << center_in_world_frame << endl;
          img_x = center_in_world_frame.x - 100;
          img_y = center_in_world_frame.y - 100;
        }
        ball_center.data.push_back(cnt_proc);
        ball_center.data.push_back(center_in_world_frame.x);
        ball_center.data.push_back(center_in_world_frame.y);
        pub_center.publish(ball_center);
        img_serve = img(cv::Rect(img_x, img_y, 200, 200));
        ball_center.data.clear();
      }
      else {
        img_serve = img(cv::Rect(T_one2ori.x, T_one2ori.y, 640, 240));
        delta = cv::Point2i(0,0);
        center_int_type = cv::Point2i(0,0);
        center = cv::Point2f(0,0);
        center_in_world_frame = cv::Point2i(-1,-1);
        ball_center.data.push_back(cnt_proc);
        ball_center.data.push_back(center_in_world_frame.x);
        ball_center.data.push_back(center_in_world_frame.y);
        pub_center.publish(ball_center);
        ball_center.data.clear();
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
/*
    num = std::to_string(ros::Time::now().toSec());
    fileName_L = path + baseName_L + std::to_string(cnt_proc) + "_" + num + ".jpg";
    cv::imwrite(fileName_L, img2, compression_params);
*/
    cv::cvtColor(img_serve, img_hsv, CV_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), img_binary);

    // Open processing
    //nh.getParam("/dynamic_bool/proc_opening", proc_opening);
    //if (proc_opening == true){
      element = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
      cv::morphologyEx(img_binary, img_binary, 2, element);
    //}

    // dilate processing
    //nh.getParam("/dynamic_bool/proc_dilate", proc_dilate);
    //if (proc_dilate == true){
      dilate(img_binary, img_binary, element);
    //}

    cv::findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // minEnclosingCircle processing
    //nh.getParam("/dynamic_bool/proc_minEnclosingCircle", proc_minEnclosingCircle);
    //if (proc_minEnclosingCircle == true){
      for (int i = 0; i < contours.size(); i++){
        double area = cv::contourArea(contours[i]);       
        if ((area > 100)/* && (radius >= 5) && (radius < 40)*/){
          //cout << "left area = " << area << endl;
          cv::minEnclosingCircle(contours[i], center, radius);
        }
        //cout << "left radius = " << radius << endl;
      }
      center_int_type.x = (int)center.x;
      center_int_type.y = (int)center.y;
      //cout << "center = " << center.x << "," << center.y << endl;
      //cout << center_int_type << endl;

      //ball_center.data.push_back(center.x);
      //ball_center.data.push_back(center.y);
      //pub_center.publish(ball_center);

      contours.clear();
      hierarchy.clear();

      //radius = 0;
      //ball_center.data.clear();
    //}

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);
  
    cnt_proc += 1;

    endd_proc = ros::Time::now().toSec();
    second_proc = endd_proc - startt_proc;
    //cout << second_proc << endl;

    //timer_isDone = nh.createTimer(ros::Duration(1.0 / 1),std::bind(&Camera_::isDone_func, this));


    //pub_leftDone.publish(isDone);
    //ros::spinOnce();

  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_left");
  ros::NodeHandle nh;

  cout << "Left camera main thread on cpu:" << sched_getcpu() << endl;

  Camera_ camera;

  thread t1(ref(camera));
/*
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  int rc = pthread_setaffinity_np(t1.native_handle(), sizeof(cpu_set_t), &cpuset);
  //cout << "rc = " << rc << endl;
  if (rc != 0){
    std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
  }
*/
  t1.join();

  ros::spin();

  return 0;
}
