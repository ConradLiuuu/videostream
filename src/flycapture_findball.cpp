#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "FlyCapture2.h"

using namespace FlyCapture2;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flycapture_findball");
  ros::NodeHandle nh;
  int H_min, H_max,S_min, S_max,V_min, V_max;

  cv::Mat img, canny_img, drawing, img_cp, img_hsv;
  cv::Mat ROI(200, 200, CV_8UC3);

  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  vector<vector<cv::Point> > contour;

  vector<vector<cv::Point> > contours_poly(contour.size());
  //vector<cv::Point2f> center(contour.size());
  //vector<float> radius(contour.size());
  cv::Point2f center;
  float radius;

  cv::Point2f cen;
  float rad;

  Error error;
  Camera camera;
  CameraInfo camInfo;
  BusManager busMgr;
  PGRGuid guid_L;

  // Connect camera
  busMgr.GetCameraFromIndex(0, &guid_L);
  error = camera.Connect(&guid_L);
  if (error != PGRERROR_OK){
    error.PrintErrorTrace();
    ROS_INFO("Failed to connect to camera");
    return false;
  }

  // Get camera info
  error = camera.GetCameraInfo(&camInfo);
  if (error != PGRERROR_OK){
    ROS_INFO("Failed to get camera info");
    return false;
  }
  cout << "Vendor name :" << camInfo.vendorName << endl;
  cout << "Model name :" << camInfo.modelName << endl;
  cout << "Serial number :" << camInfo.serialNumber << endl;

  // Start capture image
  error = camera.StartCapture();
  if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error != PGRERROR_OK){
    ROS_INFO("Failed to start image capture");
    return false;
  }

  // capture loop

  Image rawImage;
  Image bgrImage;

  unsigned int rowBytes;

  Error err;
  char key = 0;

    nh.getParam("/dynamic_HSV_server/H_min_L",H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L",H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L",S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L",S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L",V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L",V_max);

  //while (key != 'q'){
  while (ros::ok()){
    err = camera.RetrieveBuffer(&rawImage);

    if (err != PGRERROR_OK){
      cout << "Captrue error" << endl;
      continue;
    }

    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);

    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

    img_hsv = img.clone();
    drawing = img.clone();
    cv::cvtColor(img_hsv, img_hsv, CV_BGR2HSV);
/*
    nh.getParam("/dynamic_HSV_server/H_min_L",H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L",H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L",S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L",S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L",V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L",V_max);
*/
    cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), img_cp);

    //img_cp = img_hsv.clone();
    //cv::Canny(img_cp, canny_img, 50, 240, 3);
    cv::findContours(img_cp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    cv::namedWindow("canny", 0);
    cv::imshow("canny", img_cp);


    for (int i = 0; i < contours.size(); i++){
      double area = cv::contourArea(contours[i]);
      //cout << "area = " << area << endl;
/*
      cv::Scalar color(255, 0, 0);
      cv::drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point(0, 0));
*/

      if (area > 20 /*&& area < 20*/){
        cout << "area = " << area << endl;
        cv::Scalar color(255, 0, 0);
        contour.push_back(contours[i]);
        for (int j = 0; j < contour.size(); j++){
          cv::drawContours(drawing, contour, j, color, 5, 8, hierarchy, 0, cv::Point(0, 0));
          //cv::approxPolyDP(cv::Mat(contour[j]), contours_poly[j], 3, true);
          cv::minEnclosingCircle(contour[j], center, radius);
          //cen = center[j];
          //rad = radius[j];
          //cout << "center = " << cen << endl << "radius = " << rad << endl;
          cout << "done" << endl;
        }
        /*
        for (int i = 0; i < contour.size(); i++){
          cv::approxPolyDP(cv::Mat(contour[i]), contours_poly[i], 3, true);
          cv::minEnclosingCircle(cv::Mat(contours_poly[i]), center[i], radius[i]);
          //cen = center[i];
          //rad = radius[i];
          //cout << "center = " << cen << endl << "radius = " << rad << endl;
          //center.erase();
        }*/
        //contour.clear();
      }
    }

    cv::namedWindow("draw contours", 0);
    cv::imshow("draw contours", drawing);
/*
    for (int i = 0; i < contour.size(); i++){
      cv::approxPolyDP(cv::Mat(contour[i]), contours_poly[i], 3, true);
      cv::minEnclosingCircle(cv::Mat(contours_poly[i]), center[i], radius[i]);
      cen = center[i];
      rad = radius[i];
    }
*/
    contours.clear();
    contour.clear();
    hierarchy.clear();
    //contours_poly.clear();
    //center.clear();
    //radius.clear();
    //cout << "center of ball = " << cen << endl;
    //cout << "radius of ball = " << rad << endl;

    //cv::circle(img_cp, cen, rad, cv::Scalar(0, 255, 0), 1, 8, 0);
/*
    cv::namedWindow("object", 0);
    cv::imshow("object", img_cp);
*/
    key = cv::waitKey(1);
  }

  camera.Disconnect();

  return 0;
}
