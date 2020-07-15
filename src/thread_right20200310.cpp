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
  Property frmRate, prop;

  // openCV setting
  cv::Mat img, img_hsv, img_binary, img_ROI, element, img_serve, img2;

  cv::Point2f center;
  cv::Point2i center_int_type;
  cv::Point2i center_last;
  cv::Point2i T_one2ori, T_two2one, delta;
  cv::Point2i center_in_world_frame;
  cv::Point2i top;
  cv::Point2i buttom;

  float radius;
  int morph_elem;
  int morph_size;

  int width;
  int height;

  // vectors setup
  vector<vector<cv::Point> > contours;

  // ros setting
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate, pub_center;
  ros::Subscriber sub, sub2;
  std_msgs::Float32 frameRate;
  std_msgs::Int64MultiArray ball_center;

  // image transport setting
  sensor_msgs::ImagePtr msg_img, msg_binary, msg_ROI;
  image_transport::Publisher pub_img, pub_binary, pub_ROI;

  // variable setting
  unsigned int cnt, cnt_proc;
  double startt_proc, endd_proc, second_proc;
  int H_min, H_max, S_min, S_max, V_min, V_max;
  int img_x, img_y;
  unsigned int roi1_width, roi1_height;
  int contour_size;

  std::string path = "/home/lab606a/dic/right/";
  std::string fileName_L;
  std::string baseName_L = "right";

  double num;
  vector<int> compression_params;

public:

  Camera_(){
    SerialNumber = 17491067;

    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("right_camera",1);
    pub_binary = it.advertise("right_camera_binary",1);
    pub_ROI = it.advertise("right_camera_ROI",1);

    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
    pub_center = nh.advertise<std_msgs::Int64MultiArray>("ball_center_right", 1, false);

    cnt_proc = 1;
    morph_elem = 0;
    morph_size = 2;

    roi1_width = 640;
    roi1_height = 200;

    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(50);

    center = cv::Point2f(0,0);
    radius = 5;

    center_int_type = cv::Point2i(0,0);
    center_last = cv::Point2i(200,100);
    T_one2ori = cv::Point2i(920,210);
    delta = cv::Point2i(0,0);

    nh.getParam("/dynamic_HSV_server/H_min_R", H_min);
    nh.getParam("/dynamic_HSV_server/H_max_R", H_max);
    nh.getParam("/dynamic_HSV_server/S_min_R", S_min);
    nh.getParam("/dynamic_HSV_server/S_max_R", S_max);
    nh.getParam("/dynamic_HSV_server/V_min_R", V_min);
    nh.getParam("/dynamic_HSV_server/V_max_R", V_max);

    // connect camera
    busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    error = camera.Connect(&guid);

    frmRate.type = FRAME_RATE;
    error = camera.GetProperty(&frmRate);
    cout << "Right camera setting frameRate = " << frmRate.absValue << endl;

    prop.type = WHITE_BALANCE;
    prop.onOff = true;
    prop.autoManualMode = false;
    prop.valueA = 644;
    prop.valueB = 914;
    error = camera.SetProperty(&prop);

    // start capture image
    error = camera.StartCapture();
    sub2 = nh.subscribe("/sampling_time_take_photo", 1, &Camera_::ShowImg, this);
  }

  ~Camera_(){
    cout << "close right camera" << endl;
    error = camera.StopCapture();
    camera.Disconnect();
  }

  void ShowImg(const std_msgs::Bool::ConstPtr& msg){
    msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img2).toImageMsg();
    pub_img.publish(msg_img);
  }

  void operator()(){
    sub = nh.subscribe("/sampling_time", 1, &Camera_::ImageProcessing, this);
  }

  void Zone_A(){
    img_x = 0;
    img_y = 0;
    width = 400;
    height = 400;
    //cout << "Zone A" << endl;
  }
  void Zone_B(){
    img_x = center_in_world_frame.x - 200;
    img_y = 0;
    width = 400;
    height = 400;
    //cout << "Zone B" << endl;
  }
  void Zone_C(){
    img_x = center_in_world_frame.x - 200;
    img_y = 0;
    width = 2048 - img_x;
    height = 400;
    //cout << "Zone C" << endl;
  }
  void Zone_D(){
    img_x = 0;
    img_y = center_in_world_frame.y - 100;
    width = 400;
    height = 400;
    //cout << "Zone D" << endl;
  }
  void Zone_E(){
    img_x = center_in_world_frame.x - 200;
    img_y = center_in_world_frame.y - 100;
    width = 400;
    height = 400;
    //cout << "Zone E" << endl;
    //delta = cv::Point2i(0,0);
    //cout << "deltaaa = " << delta << endl;
  }
  void Zone_F(){
    img_x = center_in_world_frame.x - 200;
    img_y = center_in_world_frame.y - 100;
    width = 2048 - img_x;
    height = 400;
    //cout << "Zone F" << endl;
  }
  void Zone_G(){
    img_x = 0;
    img_y = center_in_world_frame.y - 100;
    width = 400;
    height = 1536 - img_y;
    //cout << "Zone G" << endl;
  }
  void Zone_H(){
    img_x = center_in_world_frame.x - 200;
    img_y = center_in_world_frame.y - 100;
    width = 400;
    height = 1536 - img_y;
    //cout << "Zone H" << endl;
  }
  void Zone_I(){
    img_x = center_in_world_frame.x - 200;
    img_y = center_in_world_frame.y - 100;
    width = 2048 - img_x;
    height = 1536 - img_y;
    //cout << "Zone I" << endl;
  }

  void ImageProcessing(const std_msgs::Bool::ConstPtr& msg){
    startt_proc = ros::Time::now().toSec();
    error = camera.RetrieveBuffer(&rawImage);
    ROS_INFO("Right camera start to do image process %d", cnt_proc);
    num = ros::Time::now().toSec();
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

    // Save image
    //fileName_L = path + baseName_L + std::to_string(cnt_proc) + "_" + std::to_string(num) + ".jpg";
    //cv::imwrite(fileName_L, img, compression_params);

    if ((center_int_type.x == 0) && (center_int_type.y == 0)){
      img_x = T_one2ori.x;
      img_y = T_one2ori.y;
      width = roi1_width;
      height = roi1_height;
      img_serve = img(cv::Rect(img_x, img_y, width, height));
      delta = cv::Point2i(0,0);
      contour_size = 0;
      center_in_world_frame = cv::Point2i(-1,-1);
      ball_center.data.push_back(cnt_proc);
      ball_center.data.push_back(center_in_world_frame.x);
      ball_center.data.push_back(center_in_world_frame.y);
      ball_center.data.push_back(contour_size);
      pub_center.publish(ball_center);
      ball_center.data.clear();
    }
    else {
      if ((center_in_world_frame.x > 0) && (center_in_world_frame.x < 1848) && (center_in_world_frame.y > 0) && (center_in_world_frame.y < 1436)){
        if (img_serve.rows == 400){
          if ((center_in_world_frame.y-100) < 0){
            if ((center_in_world_frame.x-200) < 0){
              Zone_A();
            }
            else{
              if  ((center_in_world_frame.x+200) > 2048){
                Zone_C();
              }
              else{
                Zone_B();
              }
            }
          }
          else{
            if ((center_in_world_frame.y+300) > 1536){
              if ((center_in_world_frame.x-200) < 0){
                Zone_G();
              }
              else{
                if ((center_in_world_frame.x+200) > 2048){
                  Zone_I();
                }
                else{
                  Zone_H();
                }
              }
            }
            else{
              if ((center_in_world_frame.x-200) < 0){
                Zone_D();
              }
              else{
                if ((center_in_world_frame.x+200) > 2048){
                  Zone_F();
                }
                else{
                  Zone_E();
                }
              }
            }
          }

        }
        else if (img_serve.cols == 640){
          if ((center_in_world_frame.y-100) >= 0){
            Zone_E();
          }
          else{
            Zone_B();
          }
        }
      }
      else {
        img_x = T_one2ori.x;
        img_y = T_one2ori.y;
        width = roi1_width;
        height = roi1_height;
        delta = cv::Point2i(0,0);
        center_int_type = cv::Point2i(0,0);
        center = cv::Point2f(0,0);
        center_in_world_frame = cv::Point2i(-1,-1);
      }
      img_serve = img(cv::Rect(img_x, img_y, width, height));
    }

    msg_ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_serve).toImageMsg();
    pub_ROI.publish(msg_ROI);

    cv::cvtColor(img_serve, img_hsv, CV_BGR2HSV_FULL);
    cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), img_binary);

    // Open processing
    element = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
    cv::morphologyEx(img_binary, img_binary, 2, element);

    // dilate processing
    //dilate(img_binary, img_binary, element);

    cv::findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //contour_size = contours.size();
    //cout << contours.size() << endl;

    // minEnclosingCircle processing
    for (int i = 0; i < contours.size(); i++){
      double area = cv::contourArea(contours[i]);
      //cout << area << endl;
      if ((area > 30)){
        cv::minEnclosingCircle(contours[i], center, radius);
        //cout << "right radius = " << radius << endl;
      }
    }
    center_int_type.x = (int)center.x;
    center_int_type.y = (int)center.y;

    //cout << center_int_type << endl;

    if (radius > 3){
      contour_size = 1;
    }
    else{
      contour_size = 0;
    }

    if ((center_int_type.x >0) && (center_int_type.y > 0) && (center_in_world_frame.y < 1336) && (radius > 3)){
      /* save image */
      //fileName_L = path + baseName_L + std::to_string(cnt_proc) + "_" + std::to_string(num) + ".jpg";
      //cv::imwrite(fileName_L, img, compression_params);

      if (img_serve.cols == 640){
        cout << "-----640------" << endl;
        //T_two2one = center_int_type - center_last;
        center_in_world_frame = center_int_type/* + T_two2one*/ + T_one2ori;
        //cout << "right center640 = " << center_in_world_frame << endl;
      }
      if (img_serve.cols == 400){
        //cout << "-----300------" << endl;
        //delta = delta + (center_int_type - center_last);
        //center_in_world_frame = center_last + T_two2one + T_one2ori + delta;
        delta = center_int_type-center_last;
        //center_last = center_last+delta;
        //cout << "delta = " << delta << endl;
        center_in_world_frame = center_in_world_frame + delta;
        //center_last = center_last - delta;
        //cout << "right center = " << center_in_world_frame << endl;
      }
      //top = center_in_world_frame-cv::Point2i(200,200);
      //buttom = center_in_world_frame+cv::Point2i(200,200);
      top.x = img_x;
      top.y = img_y;
      buttom.x = top.x + width;
      buttom.y = top.y + height;
      img2 = img.clone();
      cv::rectangle(img2, top, buttom, cv::Scalar(0,0,255), 3, 8, 0);
    }
    else {
      delta = cv::Point2i(0,0);
      center_int_type = cv::Point2i(0,0);
      //center_last = cv::Point2i(200,100);
      center = cv::Point2f(0,0);
      center_in_world_frame = cv::Point2i(-1,-1);
      img2 = img.clone();
    }
    ball_center.data.push_back(cnt_proc);
    ball_center.data.push_back(center_in_world_frame.x);
    ball_center.data.push_back(center_in_world_frame.y);
    ball_center.data.push_back(contour_size);
    pub_center.publish(ball_center);
    ball_center.data.clear();
    contours.clear();

    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
    pub_binary.publish(msg_binary);

    cnt_proc += 1;

    endd_proc = ros::Time::now().toSec();
    second_proc = endd_proc - startt_proc;
    //cout << "right processing time = " << second_proc << endl;
    //cout << second_proc << endl;
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_right");
  ros::NodeHandle nh;

  cout << "Right camera main thread on cpu:" << sched_getcpu() << endl;

  Camera_ camera;

  thread t1(ref(camera));
/*
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(5, &cpuset);
  int rc = pthread_setaffinity_np(t1.native_handle(), sizeof(cpu_set_t), &cpuset);
  //cout << "rc = " << rc << endl;
  if (rc != 0){
    std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
  }
*/
  //t1.join();
  //cout << "joined" << endl;

  ros::spin();

  t1.join();
  //cout << "joined" << endl;

  return 0;
}
