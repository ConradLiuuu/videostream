 #include <ros/ros.h>
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 using namespace cv;

 ros::Publisher pub;
 void transformer(const sensor_msgs::Image::ConstPtr& img)
 {
      cv_bridge::CvImagePtr cvimg = cv_bridge::toCvCopy(img , sensor_msgs::image_encodings::BGR8);
      //To Grayscale
      cv::Mat img_conv;
      cv::cvtColor(cvimg->image,img_conv,CV_BGR2GRAY);
      
      //Publish it
      cvimg->image = img_conv;
      cvimg->encoding = "mono8"; //這一行超容易忘記= =
      pub.publish(cvimg->toImageMsg());
 }

 int main(int argc, char** argv)
 {
      ros::init(argc, argv, "color_to_gray_node");
      ros::NodeHandle node;

      pub = node.advertise("/usb_cam/image_mono", 1000);
      ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 1, transformer);

      ros::spin();
}
