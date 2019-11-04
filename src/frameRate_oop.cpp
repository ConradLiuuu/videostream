#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <ctime>

using namespace FlyCapture2;
using namespace std;
using namespace ros;

class Camera_R
{
private:
  // ros setting
  ros::NodeHandle nh;
  Publisher pub_frameRate_right;
  std_msgs::Float32 frameRate_right;
  Timer timer_str;

  // image transport setting
  image_transport::Publisher pub_right;
  sensor_msgs::ImagePtr msg_right;

  // flycapture setting
  unsigned int SerialNumber_R;
  BusManager busMgr_R;
  Error error_R;
  Camera camera_R;
  CameraInfo camInfo_R;
  PGRGuid guid_R;
  Image rawImage_R;
  Image bgrImage_R;
  unsigned int rowBytes_R;
  Property frmRate_R;
  // openCV setting
  cv::Mat img_R;

  // other setting
  double pub_freq; // Hz
  clock_t start_R, end_R;
  int cnt;
  //int num_frames_R;
  double second, fps;

public:
  Camera_R(){
    pub_freq = 7.0;
    int cnt = 1;
    image_transport::ImageTransport it(nh);
    pub_right = it.advertise("right_camera",1);
    pub_frameRate_right = nh.advertise<std_msgs::Float32>("frameRate_right",1,false);
    //timer_str = nh.createTimer(ros::Duration(1.0 / pub_freq),std::bind(&Camera_R::cap, this));
    SerialNumber_R = 17491067;

    // connect camera
    busMgr_R.GetCameraFromSerialNumber(SerialNumber_R, &guid_R);
    error_R = camera_R.Connect(&guid_R);
    error_R = camera_R.Connect(&guid_R);
    if (error_R != PGRERROR_OK){
      ROS_INFO("Failed to connect to right camera");
      //return false;
    }
    // start capture image
    error_R = camera_R.StartCapture();

    frmRate_R.type = FRAME_RATE;
    error_R = camera_R.GetProperty(&frmRate_R);
    //ROS_INFO("Right camera setting frameRate = %d ", (int)frmRate_R.absValue);
    cout << "Right camera setting frameRate = " << frmRate_R.absValue << endl;

    cap();
  }

  void cap(){
    Rate rate(30);
    while (ros::ok()){
      if (cnt == 1){
        start_R = clock(); // start count time
      }
      error_R = camera_R.RetrieveBuffer(&rawImage_R);
      rawImage_R.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_R);
      rowBytes_R = (double)bgrImage_R.GetReceivedDataSize() / (double)bgrImage_R.GetRows();
      img_R = cv::Mat(bgrImage_R.GetRows(), bgrImage_R.GetCols(), CV_8UC3, bgrImage_R.GetData(), rowBytes_R);

      msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_R).toImageMsg();
      pub_right.publish(msg_right);
      ros::spinOnce();
      rate.sleep();

      cnt += 1;
      //cout << cnt << endl;

      if (cnt == 121){
        end_R = clock(); // stop count time
        second = (double) difftime(end_R, start_R) / CLOCKS_PER_SEC;
        fps = (double) frmRate_R.absValue / (second);

        //ROS_INFO("Taken time = %f", second);
        //ROS_INFO("Estimate frame rate = %f", fps);

        frameRate_right.data = fps;
        pub_frameRate_right.publish(frameRate_right);
        cout << /*"Taken time of right camera:" <<*/ second << endl;

        cnt = 1;
      }
    }
  }

};

int main(int argc, char** argv)
{
  init(argc, argv, "frameRate_oop");

  Camera_R camera_R;
  spin();

  return 0;
}
