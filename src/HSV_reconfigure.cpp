#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <string>
using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGB2HSV");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_hsv = it.advertise("hsv",1);
  image_transport::Publisher pub_img = it.advertise("origin",1);
  image_transport::Publisher pub_bin = it.advertise("binary",1);
  ros::Rate rate(5);
  int H_min, H_max,S_min, S_max,V_min, V_max;

  std::string path;
  nh.getParam("/picture_path", path);

  Mat img = imread(path);

  sensor_msgs::ImagePtr msg_hsv, msg_origin, msg_bin;

  Mat frame;
  Mat threshold = Mat::zeros(img.rows, img.cols, CV_8U);

  frame = img.clone();
  cvtColor(frame, frame, CV_BGR2HSV);

  msg_hsv = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  msg_origin = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  ROS_INFO("Publishing image");
  //imshow("origin", img);
  //imshow("hsv", frame);
  //waitKey();
  
  while(ros::ok()){
    nh.getParam("/dynamic_HSV_server/H_min_L",H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L",H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L",S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L",S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L",V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L",V_max);

    inRange(frame, Scalar(H_min, S_min, V_min), Scalar(H_max, S_max, V_max), threshold);

    msg_bin = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();

    pub_hsv.publish(msg_hsv);
    pub_img.publish(msg_origin);
    pub_bin.publish(msg_bin);
    //imshow("binary", threshold);
    //waitKey();
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
