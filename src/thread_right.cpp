#include "videostream/camera_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_right");
  ros::NodeHandle nh;

  cout << "Right camera main thread on cpu:" << sched_getcpu() << endl;

  CameraNode camera("right", 17491067);

  thread t1(ref(camera));

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(5, &cpuset);
  int rc = pthread_setaffinity_np(t1.native_handle(), sizeof(cpu_set_t), &cpuset);
  //cout << "rc = " << rc << endl;
  if (rc != 0){
    std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
  }

  t1.join();

  ros::spin();

  return 0;
}
