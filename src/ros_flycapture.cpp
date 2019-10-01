#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "FlyCapture2.h"

using namespace FlyCapture2;
//using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_flycapture");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_img = it.advertise("origin",1);
  ros::Rate rate(10);
  sensor_msgs::ImagePtr msg_origin;

  Error error;
  Camera camera;
  CameraInfo camInfo;

  // Connect camera
  error = camera.Connect(0);
  if (error != PGRERROR_OK){
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
  cv::Mat img;

  Error err;
  char key = 0;

  //while (key != 'q'){
  while (ros::ok()){
    // Get image
    //Image rawImage;
    Error err = camera.RetrieveBuffer(&rawImage);

    if (err != PGRERROR_OK){
      ROS_INFO("Captrue error");
      continue;
    }
    
    // Convert to bgr
    //Image bgrImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);

    // Conver to OpenCV Mat
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

    ROS_INFO("rows = %d", bgrImage.GetRows());
    ROS_INFO("cols = %d", bgrImage.GetCols());

    //cv::resize(img, img, cv::Size(), 0.315, 0.315);

    msg_origin = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub_img.publish(msg_origin);
    ros::spinOnce();
    rate.sleep();

    //cv::namedWindow("image", 0);
    //cv::imshow("image", img);
    //key = cv::waitKey(30);
  }

  //error = camera.StopCapture();
  camera.Disconnect();

  return 0;
}
