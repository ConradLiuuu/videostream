#include "videostream/camera_node_gpu.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thread_right_gpu");
    ros::NodeHandle nh;

    cout << "Right camera main thread on cpu:" << sched_getcpu() << endl;

    CameraNode camera("left", 17491067);

    thread Image_proc(ref(camera));

    /*
    // Specify cpu
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(5, &cpuset);
    int rc = pthread_setaffinity_np(t1.native_handle(), sizeof(cpu_set_t), &cpuset);
    //cout << "rc = " << rc << endl;
    if (rc != 0){
        std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
    }
    */

   spin();

   Image_proc.join();

    return 0;
}