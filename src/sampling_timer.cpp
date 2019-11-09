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
    freq_pub = 3;
    msg.data = true;
    //ros::Rate rate(10);
    chatter = nh.advertise<std_msgs::Bool>("/sampling_time", 1);
    timer_bool = nh.createTimer(ros::Duration(1.0 / freq_pub),std::bind(&Talker::pub_bool, this));
/*
    ROS_INFO("Published trigger sign");
    for (int i = 0; i < 5; i++){
      chatter.publish(msg);
      //ROS_INFO("Published trigger sign");
      rate.sleep();
    }
    //ROS_INFO("Published trigger sign");

    pub_bool();*/
  }

  void pub_bool()
  {
    chatter.publish(msg);
    //ROS_INFO("Publishe loop");
/*
    while(ros::ok()){
      sub_left = nh.subscribe("/left_camera_done", 1, &Talker::callback_left, this);
      spinOnce();
      sub_right = nh.subscribe("/right_camera_done", 1, &Talker::callback_right, this);
      spinOnce();
      //sub_left = nh.subscribe("/left_camera_done", 1, &Talker::callback_left, this);
      if (left_isDone == true && right_isDone == true){
        //ROS_INFO("sub both two");
        chatter.publish(msg);
      }
    }
*/
  }

  void callback_left(const std_msgs::Bool::ConstPtr& msg_left)
  {
    ROS_INFO("left called");
    left_isDone = msg_left->data;
  }

  void callback_right(const std_msgs::Bool::ConstPtr& msg_right)
  {
    ROS_INFO("right called");
    right_isDone = msg_right->data;
  }

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampling_timer");
  Talker talker;
  spin();
  return 0;
}
