#include <ros/ros.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  Mat img, canny_img;
  int thresh = 100;

  std::string path = "/home/liu/catkin_ws/src/videostream/picrure/tabletennis_binary.jpg";
  img = imread(path);

  Canny( img, canny_img, thresh, thresh*2, 3 );

  namedWindow("Canny", 0);
  imshow("Canny", canny_img);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  findContours( canny_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  Mat drawing = imread("/home/liu/catkin_ws/src/videostream/picrure/tabletennis.jpg");
  Mat bgr = drawing.clone();
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 0, 0 );
       drawContours( drawing, contours, i, color, 10, 8, hierarchy, 0, Point() );
     }
  namedWindow("draw", 0);
  imshow("draw", drawing);

  Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC3);
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 255 );
       drawContours( mask, contours, i, color, CV_FILLED);
     }
  namedWindow("mask", 0);
  imshow("mask", mask);

  Mat obj;
  bitwise_and(bgr, mask, obj);
  namedWindow("object", 0);
  imshow("object", obj);

  waitKey();

  return 0;
}
