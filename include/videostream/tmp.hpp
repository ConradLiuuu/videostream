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
using namespace ros;

using std::string;
using std::cout;
using std::endl;
using std::vector;

class CameraNode
{
 public:
  

 private:
  // flycapture setting
  unsigned int serial_number_, row_bytes_;
  BusManager busMgr_;
  Error error;
  Camera camera_;
  CameraInfo cam_info_;
  PGRGuid guid_;
  Image raw_image_;
  Image bgr_image_;
  Property frm_rate_;

  // openCV setting
  int morph_element_, morph_size_;
  double radius_;

  cv::Mat img_, img_hsv_, img_binary_, img_roi_, img_serve_;
  cv::Mat element_;

  // vectors setup
  vector<vector<cv::Point>> contour_, contours_;
  vector<cv::Vec4i> hierarchy_;

  cv::Point2f center_, center_last_;
  cv::Point2f t_one2ori_, t_two2ori_, delta_;
  cv::Point2f center_in_world_;

  // ros setting
  ros::NodeHandle nh_;
  ros::Publisher pub_frame_rate_, pub_binary_frame_rate_, pub_center_, pub_done_;
  ros::Subscriber sub_;
  std_msgs::Float32 frame_rate_, binary_frame_rate_;
  std_msgs::Float64MultiArray ball_center_;
  std_msgs::Bool is_done_;
  ros::Timer timer_;

  // image transport setting
  sensor_msgs::ImagePtr msg_img_, msg_binary_, msg_roi_;
  image_transport::Publisher pub_img_, pub_binary_, pub_roi_;
};