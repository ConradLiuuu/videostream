#include "FlyCapture2.h"

#include <opencv2/opencv.hpp>

#include <iostream>
//#include <time.h>
#include <ctime>

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

    Property frmRate;
    frmRate.type = FRAME_RATE;
    Error err = camera.GetProperty(&frmRate);
    cout << "Setting frameRate = " << frmRate.absValue << endl;

    double fps;
    //time_t start, end;
    clock_t start, end;
    double second;
    int num_frames = frmRate.absValue;
    int dif;
    clock_t s,d;

    while(key != 'q')
    {
      //time(&start);
      start = clock();
      //s = clock();
      Error error = camera.RetrieveBuffer( &rawImage );
      //d = clock();
      //cout << "buffer time:" << d-s << endl;
      
      for (int i = 0; i  < num_frames; i++){
        // Get the image
        //Image rawImage;
        
        //Error error = camera.RetrieveBuffer( &rawImage );
        /*
        if ( error != PGRERROR_OK )
        {
                std::cout << "capture error" << std::endl;
                continue;
        }
        */
        // convert to rgb
        //Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        //cv::namedWindow("image", 0);
        cv::imshow("image", image);
      }
      //time(&end);
      end = clock();
      //dif = end - start;
      //second = difftime(end, start);
      second = dif/CLOCKS_PER_SEC;
      //cout << "Time taken:" << second << " sec" << endl;
      dif = end - start;
      cout << dif << endl;
      cout << CLOCKS_PER_SEC << endl;

      //fps = num_frames / second;
      fps = num_frames * CLOCKS_PER_SEC / dif;
      cout << "Estimated frame rate:" << fps << endl;

        //cv::imshow("image", image);
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

