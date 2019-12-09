#include "ros/ros.h"
#include "std_msgs/Bool.h"

using namespace ros;
/*
void callback(){
  timer.publish(msg);
}

int main(int argc, char **argv)
{
    init(argc, argv, "talker");
    NodeHandle nh;

    Timer timerrr;

    Publisher timer = nh.advertise<std_msgs::Bool>("sampling_time", 1);
    std_msgs::Bool msg;

    //Rate rate(3);
    msg.data = true;
    while (ros::ok()){
      timerrr = nh.createTimer(ros::Duration(1.0 / 100),std::bind(callback));
      
      spinOnce();
      //rate.sleep();
    }
    return 0;
}
*/
bool left_isDone, right_isDone;

class Talker
{
private:
  NodeHandle nh;
  Publisher chatter;
  Subscriber sub_left, sub_right;
  Timer timer_bool;
  std_msgs::Bool msg;

  double freq_pub;
public:
  Talker()
  {
    freq_pub = 30;
    msg.data = true;
    //ros::Rate rate(10);
    chatter = nh.advertise<std_msgs::Bool>("/sampling_time", 1);
    timer_bool = nh.createTimer(ros::Duration(1.0 / freq_pub),std::bind(&Talker::pub_bool, this));
  }

  void pub_bool()
  {
    chatter.publish(msg);
  }

};
class Takephoto
{
private:
  NodeHandle nh;
  Publisher chatter;
  Subscriber sub_left, sub_right;
  Timer timer_bool;
  std_msgs::Bool msg;

  double freq_pub;
public:
  Takephoto()
  {
    freq_pub = 121;
    msg.data = true;
    //ros::Rate rate(10);
    chatter = nh.advertise<std_msgs::Bool>("/sampling_time_take_photo", 1);
    timer_bool = nh.createTimer(ros::Duration(1.0 / freq_pub),std::bind(&Takephoto::pub_Takephoto, this));
  }

  void pub_Takephoto()
  {
    chatter.publish(msg);
  }

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampling_timer");
  Talker talker;
  Takephoto takephoto;
  spin();
  return 0;
}
