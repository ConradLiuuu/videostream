#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
using namespace cv;
using namespace std;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"ros_videocapture");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("CameraStream",10);

  VideoCapture cap(0);//Capture Image from /dev/video0

  /*if(!cap.isOpened()){
  ROS_INFO("Can't open webcam");
  return -1;
  }*/

  sensor_msgs::ImagePtr msg;
  //sensor_msgs::CompressedImage msg;
  //ros::Rate loop_rate(0.1);
  //namedWindow("CameraStream",WINDOW_AUTOSIZE);
  ROS_INFO("Camera is ready");

  while(ros::ok()){
    //Mat frame(240, 320,CV_8S,Scalar(100));
    Mat frame;//not setting arguments
    cap >> frame;
    //msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
    //pub.publish(msg);//publish msg to topic
    //imshow("CameraStream",frame);
    //cvtColor(frame, edges, CV_BGR2GRAY);
    if(!frame.empty()){
      msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
      //imshow("CameraStream",frame);//show image on PC
      pub.publish(msg);//publish msg to topic
      //waitKey(0.1);
    int c;
    c = waitKey(1);
    
      if(c == 27) {
        break;
      }
    }    
    ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}


