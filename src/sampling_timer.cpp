#include "ros/ros.h"
#include "std_msgs/Bool.h"

using namespace ros;

class Sampling
{
private:
  NodeHandle nh;
  Publisher pub;
  Timer timer;
  std_msgs::Bool msg;
  double freq_pub;
public:
  Sampling()
  {
    freq_pub = 60;
    msg.data = true;
    pub = nh.advertise<std_msgs::Bool>("/sampling_time", 1);
    timer = nh.createTimer(ros::Duration(1.0 / freq_pub),std::bind(&Sampling::pub_bool, this));
  }
  void pub_bool()
  {
    ROS_INFO("Sending trigger.");
    pub.publish(msg);
  }
};

class Takephoto
{
private:
  NodeHandle nh;
  Publisher pub;
  Timer timer;
  std_msgs::Bool msg;
  double freq_pub;
public:
  Takephoto()
  {
    freq_pub = 30;
    msg.data = true;
    pub = nh.advertise<std_msgs::Bool>("/sampling_time_take_photo", 1);
    timer = nh.createTimer(ros::Duration(1.0 / freq_pub),std::bind(&Takephoto::pub_Takephoto, this));
  }
  void pub_Takephoto()
  {
    pub.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampling_timer");
  Sampling sampling;
  Takephoto takephoto;
  spin();
  return 0;
}
