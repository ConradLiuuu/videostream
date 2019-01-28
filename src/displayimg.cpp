#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char** argv)
{
  Mat img;
  img = imread("/home/gtliu/IMG.JPG");

  namedWindow("Display Image",WINDOW_AUTOSIZE);
  imshow("Display Image",img);

  waitKey(0);
  //destroyWindow("IMG");

  return 0;
}

