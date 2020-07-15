#include "FlyCapture2.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <iostream>
#include <vector>

using namespace FlyCapture2;
using namespace std;
using namespace ros;

class Camera_
{
private:
    unsigned int SerialNumber;
    unsigned int rowBytes;
    BusManager busMgr;
    Error error;
    Camera camera;
    CameraInfo camInfo;
    PGRGuid guid;
    Image rawImage;
    Image bgrImage;
    Property frmRate_set, prop, frmRate_get;

    cv::Mat img, img_hsv, img_binary, img_ROI, element, img_serve, img2;
    cv::cuda::GpuMat uimg_src, uimg_dst, med_src, med_dst;

    cv::Point2f center;
    cv::Point2i center_int_type, center_last, center_in_world_frame;
    cv::Point2i T_one2ori, T_two2one, delta;
    cv::Point2i top, buttom;
    vector<vector<cv::Point> > contours;

    float radius, width, height;
    int morph_elem, morph_size;

    NodeHandle nh;
    Publisher pub_center;
    Subscriber sub, sub2;
    std_msgs::Int64MultiArray ball_center;

    sensor_msgs::ImagePtr msg_img;
    sensor_msgs::ImagePtr msg_binary;
    sensor_msgs::ImagePtr msg_ROI;
    image_transport::Publisher pub_img;
    image_transport::Publisher pub_binary;
    image_transport::Publisher pub_ROI;

    unsigned int cnt_proc;
    int H_min, H_max, S_min, S_max, V_min, V_max;
    int img_x, img_y;
    unsigned int roi1_width, roi1_height;
    int contour_size;
    double area;
    double start_, end_;

    std::string path = "/home/lab606a/dic/right/";
    std::string fileName;
    std::string baseName = "right";
    double num;
    vector<int> compression_params;


public:
    Camera_(){
        image_transport::ImageTransport it(nh);
        pub_img = it.advertise("right_camera",1);
        pub_binary = it.advertise("right_camera_binary",1);
        pub_ROI = it.advertise("right_camera_ROI",1);

        pub_center = nh.advertise<std_msgs::Int64MultiArray>("ball_center_right", 1, false);
        sub2 = nh.subscribe("/sampling_time_take_photo", 1, &Camera_::ShowImg, this);

        morph_elem = 0;
        morph_size = 2;
        roi1_width = 640;
        roi1_height = 200;
        radius = 5;

        center = cv::Point2f(0,0);
        center_int_type = cv::Point2i(0,0);
        center_last = cv::Point2i(200,100);
        T_one2ori = cv::Point2i(920,210);
        delta = cv::Point2i(0,0);

        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(50);

        cnt_proc = 1;
        nh.getParam("/dynamic_HSV_server/H_min_R", H_min);
        nh.getParam("/dynamic_HSV_server/H_max_R", H_max);
        nh.getParam("/dynamic_HSV_server/S_min_R", S_min);
        nh.getParam("/dynamic_HSV_server/S_max_R", S_max);
        nh.getParam("/dynamic_HSV_server/V_min_R", V_min);
        nh.getParam("/dynamic_HSV_server/V_max_R", V_max);

        SerialNumber = 17491067;
        busMgr.GetCameraFromSerialNumber(SerialNumber, &guid);
        error = camera.Connect(&guid);

        frmRate_set.type = FRAME_RATE;
        frmRate_set.onOff = true;
        frmRate_set.autoManualMode = false;
        frmRate_set.absControl = true;
        frmRate_set.absValue = 120;
        error = camera.SetProperty(&frmRate_set);

        frmRate_get.type = FRAME_RATE;
        error = camera.GetProperty(&frmRate_get);
        cout << "Right camera setting frameRate = " << frmRate_get.absValue << endl;

        prop.type = WHITE_BALANCE;
        prop.onOff = true;
        prop.autoManualMode = false;
        prop.valueA = 644;
        prop.valueB = 955;
        error = camera.SetProperty(&prop); 
        error = camera.StartCapture();
    }

    ~Camera_(){
        cout << "Closing right camera" << endl;
        error = camera.StopCapture();
        camera.Disconnect();
    }
    
    void ShowImg(const std_msgs::Bool::ConstPtr& msg){
        msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img2).toImageMsg();
        pub_img.publish(msg_img);
    }

    void operator()(){
        sub = nh.subscribe("/sampling_time", 1, &Camera_::ImageProcessing, this);
    }

    void Zone_A(){
        img_x = 0;
        img_y = 0;
        width = 400;
        height = 400;
        //cout << "Zone A" << endl;
    }
    void Zone_B(){
        img_x = center_in_world_frame.x - center_last.x;
        img_y = 0;
        width = 400;
        height = 400;
        //cout << "Zone B" << endl;
    }
    void Zone_C(){
        img_x = center_in_world_frame.x - center_last.x;
        img_y = 0;
        width = 2048 - img_x;
        height = 400;
        //cout << "Zone C" << endl;
    }
    void Zone_D(){
        img_x = 0;
        img_y = center_in_world_frame.y - center_last.y;
        width = 400;
        height = 400;
        //cout << "Zone D" << endl;
    }
    void Zone_E(){
        img_x = center_in_world_frame.x - center_last.x;
        img_y = center_in_world_frame.y - center_last.y;
        width = 400;
        height = 400;
        //cout << "Zone E" << endl;
    }
    void Zone_F(){
        img_x = center_in_world_frame.x - center_last.x;
        img_y = center_in_world_frame.y - center_last.y;
        width = 2048 - img_x;
        height = 400;
        //cout << "Zone F" << endl;
    }
    void Zone_G(){
        img_x = 0;
        img_y = center_in_world_frame.y - center_last.y;
        width = 400;
        height = 1536 - img_y;
        //cout << "Zone G" << endl;
    }
    void Zone_H(){
        img_x = center_in_world_frame.x - center_last.x;
        img_y = center_in_world_frame.y - center_last.y;
        width = 400;
        height = 1536 - img_y;
        //cout << "Zone H" << endl;
    }
    void Zone_I(){
        img_x = center_in_world_frame.x - center_last.x;
        img_y = center_in_world_frame.y - center_last.y;
        width = 2048 - img_x;
        height = 1536 - img_y;
        //cout << "Zone I" << endl;
    }

    void pub_center_(){
        ball_center.data.push_back(cnt_proc);
        ball_center.data.push_back(center_in_world_frame.x);
        ball_center.data.push_back(center_in_world_frame.y);
        pub_center.publish(ball_center);
        ball_center.data.clear();
    }

    void not_found_ball(){
        reset();

        contour_size = 0;
        img_serve = img(cv::Rect(img_x, img_y, width, height));
        pub_center_();
    }

    void reset(){
        img_x = T_one2ori.x;
        img_y = T_one2ori.y;
        width = roi1_width;
        height = roi1_height;
        delta = cv::Point2i(0,0);
        center_in_world_frame = cv::Point2i(-1,-1);
    }

    void find_ball(){
        uimg_src.upload(img_serve);
        cv::cuda::cvtColor(uimg_src, uimg_dst, CV_BGR2HSV_FULL);
        uimg_dst.download(img_hsv);

        cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), img_binary);
        cv::medianBlur(img_binary, img_binary, 5);
        cv::findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        for (int i = 0; i < contours.size(); i++){
            area = cv::contourArea(contours[i]);
            if ((area > 50)){
                cv::minEnclosingCircle(contours[i], center, radius);
            }
        }
        
        center_int_type.x = (int)center.x;
        center_int_type.y = (int)center.y;

        calculate_in_world();
    }

    void calculate_in_world(){
        if ((center_int_type.x >0) && (center_int_type.y > 0) && (center_in_world_frame.y < 1336) && (radius > 3)){
            if (img_serve.cols == 640){
                center_in_world_frame = center_int_type + T_one2ori;
            }
            if (img_serve.cols == 400){
                delta = center_int_type-center_last;
                center_in_world_frame = center_in_world_frame + delta;
                cout << "right center = " << center_in_world_frame << endl;
            }
            set_track_window();
        }
        else{
            delta = cv::Point2i(0,0);
            center_int_type = cv::Point2i(0,0);
            center = cv::Point2f(0,0);
            center_in_world_frame = cv::Point2i(-1,-1);
            img2 = img.clone();
        }
        pub_center_();
        contours.clear();

        msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();
        pub_binary.publish(msg_binary);
    }

    void set_track_window(){
        top.x = img_x;
        top.y = img_y;
        buttom.x = top.x + width;
        buttom.y = top.y + height;
        img2 = img.clone();
        cv::circle(img2, center_in_world_frame, radius, cv::Scalar(0,255, 0), 3, 8, 0);
        cv::rectangle(img2, top, buttom, cv::Scalar(0,0,255), 3, 8, 0);
    }
    
    void ImageProcessing(const std_msgs::Bool::ConstPtr& msg){
        ROS_INFO("Right camera starts to do image process %d.", cnt_proc);
        start_ = ros::Time::now().toSec();
        error = camera.RetrieveBuffer(&rawImage);
        //ROS_INFO("Right camera starts to do image process %d.", cnt_proc);
        num = ros::Time::now().toSec();

        rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
        rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
        img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);

        if ((center_int_type.x == 0) && (center_int_type.y == 0)){
            not_found_ball();
        }
        else{
            if ((center_in_world_frame.x > 0) && (center_in_world_frame.x < 1848) && (center_in_world_frame.y > 0) && (center_in_world_frame.y < 1436)){
                if (img_serve.rows == 400){
                    if ((center_in_world_frame.y - center_last.y) < 0){
                        if ((center_in_world_frame.x-200) < 0){
                            Zone_A();
                        }
                        else{
                            if ((center_in_world_frame.x+200) > 2048){
                                Zone_C();
                            }
                            else
                            {
                                Zone_B();
                            }
                        }
                    }
                    else{
                        if ((center_in_world_frame.y+300) > 1536){
                            if ((center_in_world_frame.x-200) < 0){
                                Zone_G();
                            }
                            else
                            {
                                if ((center_in_world_frame.x+200) > 2048){
                                    Zone_I();
                                }
                                else{
                                    Zone_H();
                                }
                            }  
                        }
                        else{
                            if ((center_in_world_frame.x-200) < 0){
                                Zone_D();
                            }
                            else{
                                if ((center_in_world_frame.x+200) > 2048){
                                    Zone_F();
                                }
                                else{
                                    Zone_E();
                                }
                            }
                        }
                    }
                }
                else if (img_serve.cols == 640){
                    if ((center_in_world_frame.y-center_last.y) >= 0){
                        Zone_E();
                    }
                    else{
                        Zone_B();
                    }
                }
            }
            else{
                reset();
                center_int_type = cv::Point2i(0,0);
                center = cv::Point2f(0,0);
            }
            img_serve = img(cv::Rect(img_x, img_y, width, height));
        }
        find_ball();
        end_ = ros::Time::now().toSec();
        cnt_proc += 1;
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thread_right_gpu");
    ros::NodeHandle nh;

    cout << "Right camera main thread on cpu:" << sched_getcpu() << endl;

    Camera_ camera;

    thread Image_proc(ref(camera));

    spin();

    Image_proc.join();

    return 0;
}