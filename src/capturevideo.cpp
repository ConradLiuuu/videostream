
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
/*
  VideoCapture cap(0);
  if(!cap.isOpened()){
  cout << "Can't open webcam" << endl;
  return -1;
  }

  Mat video;
  namedWindow("stream video",WINDOW_AUTOSIZE);
  int width,height,fourcc,frames,fps;
  width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  //frames = cap.get(CV_CAP_PROP_POS_FRAMES);
  fps = cap.get(CV_CAP_PROP_FPS);
  //fourcc = CV_FOURCC('H','2','6','4');
  cout << "width = " << width << endl;
  cout << "height = " << height << endl;
  //cout << "fourcc = " << fourcc << endl;
  //cout << "frames = " << frames << endl;
  cout << "fps = " << fps << endl;
  //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));
  cap.set(CV_CAP_PROP_FRAME_WIDTH , 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT , 480);
  while(true) {
      //fourcc = cap.get(CAP_PROP_FOURCC);
  //cout << "width = " << width;
  //cout << "height = " << height;
  //cout << "fourcc = " << fourcc;
    cout << "working" << endl;
    
    Mat frame;
    cap >> frame;

    if(!cap.read(frame)){
      cout << "can't read frame" << endl;
    }
    //cvtColor(frame,frame,CV_BGR2GRAY);
    imshow("stream video",frame);
    int c;
    c = waitKey(1);
    
    if(c == 27) {
      break;
    }
  }

  cap.release(); //turn off camera lens
  destroyAllWindows();  //close windows
*/
VideoCapture cap(0);
if(!cap.isOpened()){
    cout << "not open \n";
    return -1;
}
// Set the video resolution to HD720 (2560*720)
cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);
cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

for(;;)
{
    //cout << "loop \n";
    Mat frame, left_image, right_image;
    // Get a new frame from camera
    cap >> frame;
    // Extract left and right images from side-by-side
    cout << "loop \n";
    left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
    right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
    cout << "loop \n";
    // Display images    
    cv::imshow("frame", frame);
    cv::imshow("left", left_image);
    cv::imshow("right", right_image);
    cv::waitKey(1);
    //if(waitKey(30) >= 0) break;
}
// Deinitialize camera in the VideoCapture destructor

  return 0;
}

/*
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  //CvCapture* cap = cvCreateCameraCapture(0);
  //VideoCapture cap(0);
  //cap.set(CV_CAP_PROP_FRAME_WIDTH , 640);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT , 480);
  namedWindow("Camera",CV_WINDOW_AUTOSIZE);
  IplImage* frame;


  while(1){
    frame = cvQueryFrame(cap);
    if(!frame) break;
    cvShowImage("Camera",frame);

    int c;
    c = waitKey(1);
    
    if(c == 27) {
      break;
    }
  }
  cvReleaseCapture(&cap);
  cvDestroyWindow( "Camera" );    
  
  return 0;
}*/
