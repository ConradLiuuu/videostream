#include <ros/ros.h>
#include <std_msgs/Bool.h>
using namespace ros;

class Listener
{
private:
  NodeHandle nh;
  Subscriber sub;

public:
  Listener()
  {
    sub = nh.subscribe("/sampling_time", 1, &Listener::callback, this);
  }

  void callback(const std_msgs::Bool::ConstPtr& msg)
  {
    ROS_INFO("sub2 called");
  }
};

int main(int argc, char **argv)
{
  init(argc, argv ,"sub2");
  Listener listener;
  spin();
  return 0;
}
