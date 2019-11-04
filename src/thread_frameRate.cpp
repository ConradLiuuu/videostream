#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <iostream>
#include <ctime>
#include <pthread.h>
#include <mutex>
#include <vector>

using namespace FlyCapture2;
using namespace std;
using namespace ros;

/*
  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;
  Image rawImage;
  Image bgrImage;
  unsigned int rowBytes;
*/
class Camera_L
{
private:
  unsigned int SerialNumber;
  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;
  Image rawImage;
  Image bgrImage;
  unsigned int rowBytes;

  cv::Mat img;
  //cv::namedWindow("image", 0);
  char key;
  //Property frmRate;

public:
  Camera_L(){
    SerialNumber = 17491073;

    // connect camera
    busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    error = camera.Connect(&guid);
    // start capture image
    error = camera.StartCapture();
  }

  void operator()(){
    key = 0;
    //cv::namedWindow("image_left", 0);
    ros::NodeHandle nh_L;
    image_transport::ImageTransport it(nh_L);
    image_transport::Publisher pub_left = it.advertise("left_camera",1);
    sensor_msgs::ImagePtr msg_left;

    ros::Publisher pub_frameRate_left = nh_L.advertise<std_msgs::Float32>("frameRate_left",1,false);
    std_msgs::Float32 frameRate_left;

    ros::Rate rate(50);

    Property frmRate_L;
    frmRate_L.type = FRAME_RATE;
    error = camera.GetProperty(&frmRate_L);
    cout << "Left camera setting frameRate = " << frmRate_L.absValue << endl;

    int num_frames_L = frmRate_L.absValue;
    double second, fps;
    clock_t start_L, end_L;

    //busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    //error = camera.Connect(&guid);

    //error = camera.StartCapture();
    cout << "thread left on cpu:" << sched_getcpu() << endl;
    while (ros::ok()){
      //cout << "thread left on cpu:" << sched_getcpu() << endl;
      start_L = clock();
      for (int i = 0; i  < num_frames_L; i++){
        //ROS_INFO("Left camera start capture image %d", i);
        error = camera.RetrieveBuffer(&rawImage);
        rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
        rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
        img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

        msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub_left.publish(msg_left);
        ros::spinOnce();
        rate.sleep();
      }
      end_L = clock();
      second = (double) difftime(end_L, start_L) / CLOCKS_PER_SEC;
      fps = (double) num_frames_L / (second);
      frameRate_left.data = fps;
      pub_frameRate_left.publish(frameRate_left);
      ROS_INFO("Left Taken time = %f", second);
      ROS_INFO("Left Estimate frame rate = %f", fps);
      //cout << "Taken time of left camera:" << second << endl;
      //rate.sleep();
      //cv::imshow("image_left", img);
      //key = cv::waitKey(1);
    }
  }
};

class Camera_R
{
private:
  unsigned int SerialNumber_R;
  BusManager busMgr_R;
  Error error_R;
  Camera camera_R;
  CameraInfo camInfo_R;
  PGRGuid guid_R;
  Image rawImage_R;
  Image bgrImage_R;
  unsigned int rowBytes_R;

  cv::Mat img_R;
  //cv::namedWindow("image", 0);
  char key_R;
  //Property frmRate;
/*
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_right = it.advertise("right_camera",1);
  sensor_msgs::ImagePtr msg_right;
*/
public:
  Camera_R(){
    SerialNumber_R = 17491067;

    // connect camera
    busMgr_R.GetCameraFromSerialNumber(SerialNumber_R, &guid_R);
    error_R = camera_R.Connect(&guid_R);
    // start capture image
    error_R = camera_R.StartCapture();
  }

  void operator()(){
    key_R = 0;
    ros::NodeHandle nh_R;

    image_transport::ImageTransport it(nh_R);
    image_transport::Publisher pub_right = it.advertise("right_camera",1);
    sensor_msgs::ImagePtr msg_right;

    ros::Publisher pub_frameRate_right = nh_R.advertise<std_msgs::Float32>("frameRate_right",1,false);
    std_msgs::Float32 frameRate_right;

    ros::Rate rate_R(50);

    Property frmRate_R;
    frmRate_R.type = FRAME_RATE;
    error_R = camera_R.GetProperty(&frmRate_R);
    cout << "Right camera setting frameRate = " << frmRate_R.absValue << endl;

    int num_frames_R = frmRate_R.absValue;
    double second_R, fps_R;
    clock_t start_R, end_R;
    //cv::namedWindow("image_right", 0);

    //busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    //error = camera.Connect(&guid);

    //error = camera.StartCapture();

    //while (key_R != 'q'){
    cout << "thread right on cpu:" << sched_getcpu() << endl;
    while (ros::ok()){
      //cout << "thread right on cpu:" << sched_getcpu() << endl;
      start_R = clock();
      for (int i = 0; i  < num_frames_R; i++){
        //ROS_INFO("Right camera start capture image %d", i);
        error_R = camera_R.RetrieveBuffer(&rawImage_R);
        rawImage_R.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_R);
        rowBytes_R = (double)bgrImage_R.GetReceivedDataSize() / (double)bgrImage_R.GetRows();
        img_R = cv::Mat(bgrImage_R.GetRows(), bgrImage_R.GetCols(), CV_8UC3, bgrImage_R.GetData(), rowBytes_R);

        msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_R).toImageMsg();
        pub_right.publish(msg_right);
        ros::spinOnce();
        rate_R.sleep();
      }
      end_R = clock();
      second_R = (double) difftime(end_R, start_R) / CLOCKS_PER_SEC;
      fps_R = (double) num_frames_R / (second_R);
      frameRate_right.data = fps_R;
      pub_frameRate_right.publish(frameRate_right);
      ROS_INFO("Right Taken time = %f", second_R);
      ROS_INFO("Right Estimate frame rate = %f", fps_R);
      //rate_R.sleep();
      //cout << "Taken time of right camera:" << second_R << endl;
      //cv::imshow("image_right", img_R);
      //key_R = cv::waitKey(1);
    }
  }
};

void camera_capture(unsigned int num){
  unsigned int SerialNumber = num;
  BusManager busMgr;
  Error error;
  Camera camera;
  CameraInfo camInfo;
  PGRGuid guid;
  Image rawImage;
  Image bgrImage;
  unsigned int rowBytes;
  //unsigned int SerialNumber = num;

  ros::NodeHandle nh_L;
  image_transport::ImageTransport it(nh_L);
  image_transport::Publisher pub_left = it.advertise("left_camera",1);
  sensor_msgs::ImagePtr msg_left;

  ros::Publisher pub_frameRate_left = nh_L.advertise<std_msgs::Float32>("frameRate_left",1,false);
  std_msgs::Float32 frameRate_left;

  ros::Rate rate(30);

  busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
  error = camera.Connect(&guid);
  error = camera.StartCapture();
  //cv::namedWindow("image", 0);

  cv::Mat img;
    Property frmRate_L;
    frmRate_L.type = FRAME_RATE;
    error = camera.GetProperty(&frmRate_L);
    cout << "Left camera setting frameRate = " << frmRate_L.absValue << endl;

    int num_frames_L = frmRate_L.absValue;
    double second, fps;
    clock_t start_L, end_L;
  //char key = 0;
  while (ros::ok()){
      start_L = clock();
      for (int i = 0; i  < num_frames_L; i++){
        error = camera.RetrieveBuffer(&rawImage);
        rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
        rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
        img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);
        msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub_left.publish(msg_left);
        ros::spinOnce();
        rate.sleep();
      }
      end_L = clock();
      second = (double) difftime(end_L, start_L) / CLOCKS_PER_SEC;
      fps = (double) num_frames_L / (second);
      frameRate_left.data = fps;
      pub_frameRate_left.publish(frameRate_left);
      ROS_INFO("Left Taken time = %f", second);
      ROS_INFO("Left Estimate frame rate = %f", fps);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_frameRate");
  ros::NodeHandle nh;

  cout << "main thread on cpu:" << sched_getcpu() << endl;

  //unsigned int n = std::thread::hardware_concurrency();
  //cout << n << " concurrent threads are supported.\n";

  //thread threads[2];

  Camera_L camera_L;
  Camera_R camera_R;

  thread t1(ref(camera_L));
  //thread t1(camera_capture, 17491073);
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
  //thread t1(camera_capture, 17491073);
  //thread::id t1_id = t1.get_id();
  //thread::id t1_id = std::this_thread::get_id();
  //cout << "t1's id: " << t1_id << '\n';

  thread t2(ref(camera_R));
/*
  cpu_set_t cpuset2;
  CPU_ZERO(&cpuset2);
  CPU_SET(5, &cpuset2);
  int rc2 = pthread_setaffinity_np(t2.native_handle(), sizeof(cpu_set_t), &cpuset2);
  //cout << "rc = " << rc << endl;
  //cout << "sizeof(cpu_set_t) = " << sizeof(cpu_set_t) << endl;
  if (rc2 != 0){
    std::cerr << "Error calling pthread_setaffinity_np: " << rc2 << "\n";
  }
*/
  //thread::id t2_id = t2.get_id();
  //thread::id t2_id = std::this_thread::get_id();
  //cout << "t2's id: " << t2_id << '\n';


  t1.join();
  t2.join();

  return 0;
}
