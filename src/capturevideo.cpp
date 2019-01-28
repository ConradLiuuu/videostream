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

  while(true) {
    Mat frame;
    cap >> frame;
    imshow("stream video",frame);
    int c;
    c = waitKey(1);
    
    /*if(c == 27) {
      break;
    }*/
  }

  cap.release(); //turn off camera lens
  destroyAllWindows();  //close windows

    /*VideoCapture video(0);
    if (!video.isOpened()){
        return -1;
    }
    Size videoSize = Size((int)video.get(CV_CAP_PROP_FRAME_WIDTH),(int)video.get(CV_CAP_PROP_FRAME_HEIGHT));
    namedWindow("video demo", CV_WINDOW_AUTOSIZE);
    Mat videoFrame;

    while(true){
        video >> videoFrame;
        if(videoFrame.empty()){
            break;
        }
        imshow("video demo", videoFrame);
        waitKey(33);
    }*/
  return 0;
}

