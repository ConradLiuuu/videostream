#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <iostream>
#include <vector>
#include <string>

using namespace FlyCapture2;
using namespace std;
using namespace ros;

cv::Mat img_L, img_R;

std::string fileName_L = "/home/lab606a/pic/left.mp4";
std::string fileName_R = "/home/lab606a/pic/right.mp4";

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

  // ros setting
  ros::NodeHandle nh;
  ros::Publisher pub_frameRate, pub_binary_frameRate, pub_center, pub_rightDone;
  ros::Subscriber sub, sub2;
  std_msgs::Float32 frameRate, binary_frameRate;
  std_msgs::Int64MultiArray ball_center;
  std_msgs::Bool isDone;
  ros::Timer timer;

  // image transport setting
  sensor_msgs::ImagePtr msg_img, msg_binary, msg_ROI;
  image_transport::Publisher pub_img, pub_binary, pub_ROI;

  // variable setting
  unsigned int cnt, cnt_proc;
  unsigned int cnt_img;
  double startt, endd, startt_proc, endd_proc;
  double second, fps, second_proc, fps_proc;

  int H_min, H_max, S_min, S_max, V_min, V_max;
  bool proc_minEnclosingCircle, proc_opening, proc_dilate;
  bool func;

  int img_x, img_y;

public:
  std::string path = "/home/lab606a/dic/right/";
  std::string fileName_L;
  std::string baseName_L = "right";

  //const string file = "right.avi";
  const string source = "right.mp4";
  const string base = "right";
  string::size_type pAt = source.find_last_of('.');
  const string NAME = source.substr(0, pAt) + base + ".mp4";

  double num;
  vector<int> compression_params;
  cv::VideoWriter vid;
  vid.open(NAME,CV_FOURCC('M','J','P','G'),60, cv::Size(2048,1536));
  //writer.open("right.mp4",CV_FOURCC('M','J','P','G'),60, cv::Size(2048,1536));
  //cv::VideoCapture cap(0);
  //cv::VideoWriter video();
  //cv::VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, Size(2048,1536));

  Camera_(){
    SerialNumber = 17491067;

    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("right_camera",1);
    pub_binary = it.advertise("right_camera_binary",1);
    pub_ROI = it.advertise("right_camera_ROI",1);

    pub_frameRate = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
    pub_binary_frameRate = nh.advertise<std_msgs::Float32>("image_processing_frameRate_right",1,false);
    pub_center = nh.advertise<std_msgs::Int64MultiArray>("ball_center_right", 1, false);
    pub_rightDone = nh.advertise<std_msgs::Bool>("right_camera_done", 1, false);

    cnt = 1;
    cnt_proc = 1;

    // connect camera
    busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
    error = camera.Connect(&guid);

    frmRate.type = FRAME_RATE;
    error = camera.GetProperty(&frmRate);
    cout << "Right camera setting frameRate = " << frmRate.absValue << endl;

    // start capture image
    error = camera.StartCapture();
    sub2 = nh.subscribe("/sampling_time_take_photo", 1, &Camera_::callback2, this);
  }

  void callback2(const std_msgs::Bool::ConstPtr& msg){
    msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub_img.publish(msg_img);

  }

  void operator()(){
    sub = nh.subscribe("/sampling_time", 1, &Camera_::callback, this);
  }


  void callback(const std_msgs::Bool::ConstPtr& msg){
    error = camera.RetrieveBuffer(&rawImage);
    //ROS_INFO("Right camera start to do image process %d", cnt_proc);
    //num = ros::Time::now().toSec();
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);
    //video.write(img);

  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_video");
  ros::NodeHandle nh;

  cout << "Right camera main thread on cpu:" << sched_getcpu() << endl;

  Camera_ camera;

  thread t1(ref(camera));

  t1.join();

  ros::spin();

  return 0;
}
