#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    imshow("view",cv_bridge::toCvShare(msg,"bgr8")->image);
    waitKey(1);
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.",msg->encoding.c_str());
  }
}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"ros_videocapture");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("CameraStream",1);

  VideoCapture cap(0);//Capture Image from /dev/video0

  if(!cap.isOpened()){
  ROS_INFO("Can't open webcam");
  return -1;
  }
  namedWindow("stream video",WINDOW_AUTOSIZE);
  sensor_msgs::ImagePtr msg;
  ROS_INFO("Camera is ready");

  while(ros::ok()){
    //Mat frame(240, 320,CV_8S,Scalar(100));
    Mat frame;//not setting arguments
    cap >> frame;
    cvtColor(frame,frame,CV_BGR2GRAY);
    imshow("stream video",frame);
    if(!frame.empty()){
      msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",frame).toImageMsg();
      //cvtColor(msg->image,conv,CV_BGR2GRAY);
      //imshow("CameraStream",frame);//show image on PC
      pub.publish(msg);//publish msg to topic
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




