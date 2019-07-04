
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
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
