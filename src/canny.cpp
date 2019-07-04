#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "canny");
  ros::NodeHandle nh;
  ros::Publisher chatter = nh.advertise<std_msgs::Float64>("error", 10);
  int width,height;
  VideoCapture cap(0);
  if(!cap.isOpened()){
  cout << "Can't open webcam" << endl;
  return -1;
  }

  width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  cout << "width = " << width << endl;
  cout << "height = " << height << endl;  

  Mat img, canny1, canny2, ROI1, ROI2,ero;
  //namedWindow("Origin", cv::WINDOW_AUTOSIZE);
  //namedWindow("ROI", cv::WINDOW_AUTOSIZE);
  //namedWindow("Canny", cv::WINDOW_AUTOSIZE);
  //namedWindow("Erode", cv::WINDOW_AUTOSIZE);

  while(true){
    cap >> img;
    if(!cap.read(img)){
      cout << "can't read frame" << endl;
    }
    ROI1 = img(Rect(0,150,600,200));
    cvtColor(ROI1, canny1, CV_BGR2GRAY);
    Canny(ROI1, canny1, 50, 240);
    ROI2 = img(Rect(0,50,600,100));
    cvtColor(ROI2, canny2, CV_BGR2GRAY);
    Canny(ROI2, canny2, 50, 240);

    //erode(canny,ero,Mat());
    Moments mu1 = moments(canny1, true);
    Point2f center1 = Point2f(mu1.m10/mu1.m00 , mu1.m01/mu1.m00);
    cout << "center1 = " << center1 << endl;
    Moments mu2 = moments(canny2, true);
    Point2f center2 = Point2f(mu2.m10/mu2.m00 , mu2.m01/mu2.m00);
    cout << "center2 = " << center2 << endl;

    double point1 = center1.x;
    cout << "x1 = " << point1 << endl;
    double point2 = center2.x;
    cout << "x2 = " << point2 << endl;
    //double point1 = 0;
    double error = point2-point1;

    std_msgs::Float64 float_msg;
    float_msg.data = error;
    chatter.publish(float_msg);
    ros::spinOnce();

    circle(ROI1, center1, 10, Scalar(0,255,0), -1);
    circle(ROI2, center2, 10, Scalar(0,0,255), -1);
    imshow("Origin", img);
    //imshow("ROI1",ROI1);
    //imshow("ROI2",ROI2);
    //imshow("Canny",canny);
    //imshow("Erode",ero);

    int c;
    c = waitKey(1);
    
    if(c == 27) {
      break;
    }
  }
  cap.release(); //turn off camera lens
  destroyAllWindows();  //close windows

  return 0;
}
