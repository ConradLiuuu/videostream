#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGB2HSV");
  ros::NodeHandle nh;

  int H_min, H_max,S_min, S_max,V_min, V_max;
  std::string path;
  nh.getParam("/picture_path", path);

  Mat img = imread(path);
  Mat img_hsv = img.clone();
  Mat threshold = Mat::zeros(img.rows, img.cols, CV_8U);

  cvtColor(img_hsv, img_hsv, CV_BGR2HSV);

  nh.getParam("/dynamic_HSV_server/H_min_L",H_min);
  nh.getParam("/dynamic_HSV_server/H_max_L",H_max);
  nh.getParam("/dynamic_HSV_server/S_min_L",S_min);
  nh.getParam("/dynamic_HSV_server/S_max_L",S_max);
  nh.getParam("/dynamic_HSV_server/V_min_L",V_min);
  nh.getParam("/dynamic_HSV_server/V_max_L",V_max);

  inRange(img_hsv, Scalar(H_min, S_min, V_min), Scalar(H_max, S_max, V_max), threshold);

  namedWindow("origin image", 0);
  namedWindow("threshold image", 0);
  imshow("origin image", img);
  imshow("threshold image", threshold);
  waitKey();
  
  return 0;
}
