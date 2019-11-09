#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using namespace ros;

float x_left;
float y_left;

void callback_left(const std_msgs::Float64MultiArray::ConstPtr& msg_left)
{
  x_left = msg_left->data[0];
  y_left = msg_left->data[1];
  ROS_INFO("x = %f", x_left);
  ROS_INFO("y = %f", y_left);
}

int main(int argc, char **argv)
{
  init(argc, argv, "positioning");
  NodeHandle nh;
  Subscriber sub_left = nh.subscribe("/ball_center_left",10,callback_left);
  spin();
  return 0;
}
