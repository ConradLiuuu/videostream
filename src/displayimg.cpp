#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char** argv)
{
  Mat lenna = imread("/home/lab606a/Pictures/512to8_0318.png");
  //img = imread("/home/gtliu/lenna.jpeg");

  namedWindow("Figure",WINDOW_NORMAL);
  imshow("Figure",lenna);

  waitKey(0);
  //destroyWindow("IMG");

  return 0;
}

