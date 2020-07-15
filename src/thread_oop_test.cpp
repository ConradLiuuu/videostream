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
  ros::Publisher pub_frameRate, pub_binary_frameRate, pub_center;
  ros::Subscriber sub;
  std_msgs::Float32 frameRate, binary_frameRate;
  std_msgs::Float64MultiArray ball_center;
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

  int img_x, img_y, one2ori_x, one2ori_y;

public:
  Camera_(){
    SerialNumber = 17491073;

    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("left_camera",1);
    pub_binary = it.advertise("left_camera_binary",1);
    pub_ROI = it.advertise("left_camera_ROI",1);

    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_left",1,false);
    pub_binary_frameRate = nh.advertise<std_msgs::Float32>("image_processing_frameRate_left",1,false);
    pub_center = nh.advertise<std_msgs::Float64MultiArray>("ball_center_left", 1, false);

    cnt = 1;
    cnt_proc = 0;
    morph_elem = 0;
    morph_size = 1;

    center = cv::Point2f(0,0);
    //center.x = 0;
    //center.y = 0;
    radius = 0;

    //T_one2ori.x = 650;
    //T_one2ori.y = 230;
    center_last = cv::Point2f(100,100);
    T_one2ori = cv::Point2f(650,230);

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
    //img_serve = img(cv::Rect(650, 230, 640, 480));
    //img_serve = img(cv::Rect(915, 265, 105, 55));
    //img_serve.release();
    /*
    if (center.x == 0 && center.y == 0){
      img_serve = img(cv::Rect(650, 230, 640, 480));
      cout << img_serve.cols << ", " << img_serve.rows << endl;
    }
    else {
      //img_serve = img(cv::Rect(650, 230, 400, 400));

      img_x = (int)center.x+650-200;
      img_y = (int)center.y+230-200;
      img_serve = img(cv::Rect(img_x, img_y, 400, 400));

      cout << img_serve.cols << ", " << img_serve.rows << endl;

    }
    */
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
    msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_serve).toImageMsg();
    pub_ROI.publish(msg_ROI);
*/
/*
    if (center.x > 0 && center.y > 0 && (center.x+200)<2048 && (center.y+200)<1536){
      cout << "found center \n ";
      img_x = (int)center.x+650-200;
      img_y = (int)center.y+230-200;
      //cout << center << endl;
      //if (img.rows > 500 && img.cols > 500){
      //img_serve = img(cv::Rect((int)center.x+650-200, (int)center.y+230-200, 400, 400)); //is able to track ball in half table
      img_serve = img(cv::Rect(img_x, img_y, 400, 400));
      //cout << img_ROI.rows << " " << img_ROI.cols << endl;
      //}
      msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_serve).toImageMsg();
      pub_ROI.publish(msg_ROI);
      //center.x = 0;
      //center.y = 0;
    }

    else if (center.x == 0 && center.y == 0){
      cout << "not found center \n";
      img_serve = img(cv::Rect(650, 230, 640, 480));
      //msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_serve).toImageMsg();
      //pub_ROI.publish(msg_ROI);
    }
*/
    cnt += 1;

    if (cnt == (int)frmRate.absValue){
      endd = ros::Time::now().toSec();
      second = endd - startt;
      fps = (double) frmRate.absValue / (second);
      frameRate.data = fps;
      pub_frameRate.publish(frameRate);

      cnt = 1;
    }
      
  }


  void operator()(){
    sub = nh.subscribe("/sampling_time", 1, &Camera_::callback, this);
  }

  void callback(const std_msgs::Bool::ConstPtr& msg){
    //ROS_INFO("Left camera start to do image process");
    //img_serve = img(cv::Rect(650, 230, 640, 480));
    startt_proc = ros::Time::now().toSec();

    if ((center.x == 0) && (center.y == 0)){
      //cout << "not find ball \n";
      img_serve = img(cv::Rect(T_one2ori.x, T_one2ori.y, 640, 240));
      delta = cv::Point2f(0,0);
      //delta.x = 0;
      //delta.y = 0;
      //center_last.x = 200;
      //center_last.y = 200;
      //cout << img_serve.cols << ", " << img_serve.rows << endl;
    }
    else {
      if ((center_in_world_frame.x < 1848) && (center_in_world_frame.y < 1336)){
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

      /*
      if (((center_in_world_frame.x+200) >= 2048) || ((center_in_world_frame.y + 200) >= 1536)){
        img_serve = img(cv::Rect(T_one2ori.x, T_one2ori.y, 640, 480));
        delta = cv::Point2f(0,0);
        center = cv::Point2f(0,0);
      }*/
    }
/*
    else if (center.x > 0 && center.y > 0 ){
      //img_serve = img(cv::Rect(650, 230, 400, 400));
      if (img_serve.cols == 640){
        //center_last.x = center.x;
        //center_last.y = center.y;
        //T_two2one.x = center.x - center_last.x;
        //T_two2one.y = center.y - center_last.y;
        T_two2one = center - center_last;
        // transform to frame of 2048*1536
        //center_in_world_frame.x = center_last.x + T_two2one.x + T_one2ori.x;
        //center_in_world_frame.y = center_last.y + T_two2one.y + T_one2ori.y;
        center_in_world_frame = center_last + T_two2one + T_one2ori;
        //one2ori_x = (int)center.x+650;
        //one2ori_y = (int)center.y+230;
        //cout << "case A \n";
        img_x = (int)center_in_world_frame.x - 200;
        img_y = (int)center_in_world_frame.y - 200;
        //cout << one2ori_x << ", " << one2ori_y << endl;
      }
      else if (img_serve.cols == 400) {
        //center_last.x = center.x;
        //center_last.y = center.y;
        //cout << center.x << ", " << center.y << endl;
        //cout << "case B \n";
        cout << "center = " << center << endl;
        cout << "center last = " << center_last << endl;

        //delta.x = delta.x + (center.x - 200);
        //delta.y = delta.y + (center.y - 200);
        delta = delta + (center - center_last);
        cout << "delta = " << delta << endl;

        //center_in_world_frame.x = 200 + T_two2one.x + T_one2ori.x + delta.x;
        //center_in_world_frame.y = 200 + T_two2one.y + T_one2ori.y + delta.y;
        center_in_world_frame = center_last + T_two2one + T_one2ori + delta;

        //center_last.x = center.x;
        //center_last.y = center.y;

        img_x = (int)center_in_world_frame.x - 200;
        img_y = (int)center_in_world_frame.y - 200;
        cout << img_x << ", " << img_y << endl;

      }

      img_serve = img(cv::Rect(img_x, img_y, 400, 400));

      //cout << img_serve.cols << ", " << img_serve.rows << endl;
    }
    */
    msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_serve).toImageMsg();
    pub_ROI.publish(msg_ROI);

    //startt_proc = ros::Time::now().toSec();
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
        if (area > 160){
        //cout << "area = " << area << endl;
          cv::minEnclosingCircle(contours[i], center, radius);
        }
      }
      //center.x = center.x + 650;
      //center.y = center.y + 230;
      //cout << "center = " << center.x << "," << center.y << endl;
      /*
      if (radius > 0){
        circle(img, center,radius,cv::Scalar(255,0,0), 3, 8,0);
      }
      */
      //ball_center.data.push_back(center.x);
      //ball_center.data.push_back(center.y);
      //pub_center.publish(ball_center);
      //center_last.x = center.x;
      //center_last.y = center.y;
      contours.clear();
      hierarchy.clear();
      //radius = 0;
      //ball_center.data.clear();
    }

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);
/*
    if (center.x > 0 && center.y > 0 ){
      //img_serve = img(cv::Rect(650, 230, 400, 400));
      if (img_serve.cols == 640){
        center_last.x = 200;
        center_last.y = 200;
        // transform to frame of 2048*1536
        one2ori_x = (int)center.x+650;
        one2ori_y = (int)center.y+230;
        //cout << "case A \n";
        img_x = (int)center.x+650-200;
        img_y = (int)center.y+230-200;
        //cout << one2ori_x << ", " << one2ori_y << endl;
      }
      else if (img_serve.cols == 400) {
        //center_last.x = center.x;
        //center_last.y = center.y;
        //cout << center.x << ", " << center.y << endl;
        //cout << "case B \n";
        img_x = (int)(center.x-center_last.x) + one2ori_x - 200;
        img_y = (int) (center.y-center_last.y) + one2ori_y - 200;
        //cout << img_x << ", " << img_y << endl;
        center_last.x = center.x;
        center_last.y = center.y;
      }

      img_serve = img(cv::Rect(img_x, img_y, 400, 400));

      //cout << img_serve.cols << ", " << img_serve.rows << endl;
    }
*/
    endd_proc = ros::Time::now().toSec();
    second_proc = endd_proc - startt_proc;
    cout << second_proc << endl;
    //fps_proc = 1 / second_proc;
    //binary_frameRate.data = fps_proc;
    //pub_binary_frameRate.publish(binary_frameRate);

  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_frameRate");
  ros::NodeHandle nh;

  cout << "main thread on cpu:" << sched_getcpu() << endl;

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
