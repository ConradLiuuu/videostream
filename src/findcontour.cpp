#include <ros/ros.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  Mat img, canny_img;
  Mat ROI(200, 200, CV_8UC3);
  int thresh = 100;

  std::string path = "/home/liu/catkin_ws/src/videostream/picrure/tabletennis_binary.jpg";
  img = imread(path);

  Canny( img, canny_img, thresh, thresh*2, 3 );
/*
  namedWindow("Canny", 0);
  imshow("Canny", canny_img);
*/
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  findContours( canny_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<vector<Point> > contourrrr;

  Mat drawing = imread("/home/liu/catkin_ws/src/videostream/picrure/tabletennis.jpg");
  Mat draw = drawing.clone();
  Mat bgr = drawing.clone();
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 255 );
       //drawContours( drawing, contours, i, color, 10, 8, hierarchy, 0, Point() );
       double area = contourArea(contours[i]);
       //cout << "area = " << area << endl;
       if (area > 100) {
         contourrrr.push_back(contours[i]);
         //cout << "contour = " << contours[i] << endl;
         drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
         cout << "draw contour" << endl;
       }
     }
  namedWindow("draw", 0);
  imshow("draw", drawing);
  //vector<vector<Point> > contours;
  //findContours( drawing, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<vector<Point> > contours_poly( contourrrr.size() );
  vector<Point2f>center( contourrrr.size() );
  vector<float>radius( contourrrr.size() );
  float rad;
  Point2f cen;

  for( int i = 0; i < contourrrr.size(); i++ )
     { approxPolyDP( Mat(contourrrr[i]), contours_poly[i], 3, true );

       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
       cout << "center of minencircle = " << center[i] << endl;
       cout << "radius of minencircle = " << radius[i] << endl;
       rad = radius[i];
       //cen.push_back(center[i]);
       cen = center[i];
     }
  for( int i = 0; i< contourrrr.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 0 );
       drawContours( draw, contours_poly, i, color, 1, 8, hierarchy, 0, Point() );
     }
  namedWindow("minenclosedcircle", 0);
  imshow("minenclosedcircle", draw);

/*
  Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC3);
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 255 );
       drawContours( mask, contours, i, color, CV_FILLED);
     }
  //namedWindow("mask", 0);
  //imshow("mask", mask);
*/
/*
  Mat obj;
  bitwise_and(bgr, mask, obj);
  namedWindow("object", 0);
  imshow("object", obj);
*/
  vector<Moments> mu(contourrrr.size());
  vector<Point2f> mc(contourrrr.size());
  for( int i = 0; i< contourrrr.size(); i++ )
    {
      mu[i] = moments(contourrrr[i], false);
    }
  for( int i = 0; i< contourrrr.size(); i++ )
   {
      mc[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
   }
  for( int i = 0; i< contourrrr.size(); i++ )
     {
       Scalar color = Scalar( 255, 0, 0 ); //worse
       circle( draw, mc[i], rad, color, 1, 8, 0 );
       circle( bgr, mc[i], 1, color, -1, 8, 0 );
       cout << "center of moment= " << mc[i] << endl;
     }
   Scalar colors = Scalar( 0, 0, 255 ); //better
   circle( draw, cen, rad, colors, 1, 8, 0 );
   circle( bgr, cen, 1, colors, -1, 8, 0 );

  namedWindow("object 2", 0);
  imshow("object 2", draw);

  namedWindow("center of circle", 0);
  imshow("center of circle", bgr);

  ROI = draw(Rect(cen.x-200, cen.y-200, 400, 400));
  imshow("ROI", ROI);

  waitKey();

  return 0;
}

