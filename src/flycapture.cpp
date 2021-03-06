#include "FlyCapture2.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <typeinfo>

using namespace FlyCapture2;
using namespace std;

int main()
{

    Error error;
    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        std::cout << "Failed to connect to camera" << std::endl;  
        return -1;
    }

    cout << "Number of cameras detected: " << numCameras << endl;


    //Error error;
    Camera camera;
    CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect(0);

    if ( error != PGRERROR_OK )
    {
        error.PrintErrorTrace();
        std::cout << "Failed to connect to camera" << std::endl;     
        return false;
    }


    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;     
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " " 
              << camInfo.sensorResolution << " "
              << camInfo.serialNumber << std::endl;
	
    error = camera.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;     
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;     
        return false;
    } 

    // capture loop
    char key = 0;
    cv::namedWindow("image", 0);
    cv::Mat image;
    unsigned int rowBytes;
    Image rawImage;
    Image rgbImage;
    Error err;

    //int videoMode, frameRate;
    VideoMode videoMode;
    //VideoMode VIDEOMODE_2048x1536Y8;
    FrameRate frameRate;
    //FrameRate FRAMERATE_120;
    bool* pSupported;

    Property frmRate;
    frmRate.type = FRAME_RATE;
    err = camera.GetProperty(&frmRate);
    cout << "Setting frameRate = " << frmRate.absValue << endl;
    //camera.SetVideoModeAndFrameRate(VideoMode, FrameRate);
    //cout << "type:" << typeid(frameRate).name() << endl;

    while(key != 'q')
    {
        //err = camera.GetVideoModeAndFrameRateInfo(videoMode, frameRate, pSupported);
        //cout << "true or not :" << pSupported << endl;
        //cout << "video mode :" << videoMode << endl;
        //cout << "frame rate = " << frameRate << endl;

        // Get the image
        //Image rawImage;
        err = camera.RetrieveBuffer( &rawImage );
        if ( err != PGRERROR_OK )
        {
                std::cout << "capture error" << std::endl;
                continue;
        }

        // convert to rgb
        //Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        //cv::namedWindow("image", 0);
        cv::imshow("image", image);
        key = cv::waitKey(1);        
    }

    error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }  

    camera.Disconnect();

    return 0;
}

