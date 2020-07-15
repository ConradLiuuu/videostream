#include <ros/ros.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  Mat img, canny_img;
  Mat ROI(200, 200, CV_8UC3);
  int thresh = 100;

  // Read image
  std::string path = "/home/lab606a/catkin_ws/src/videostream/picrure/left466.jpg";
  img = imread(path);

  //Canny( img, canny_img, thresh, thresh*2, 3 );
/*
  namedWindow("Canny", 0);
  imshow("Canny", canny_img);
*/
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  // Convert color space and find contour
  cvtColor(img, img, CV_BGR2HSV);
  inRange(img, Scalar(10, 130, 200), Scalar(30, 200, 255), img);
  //Canny( img, canny_img, thresh, thresh*2, 3 );
  findContours( img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  //Canny( img, canny_img, thresh, thresh*2, 3 );
  //Canny( img, img, thresh, thresh*2, 3 );
/*
  namedWindow("binary", 0);
  imshow("binary", img);
*/
  // Container for area of contour > 2
  vector<vector<Point> > contourrrr;
/*
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );
  float rad;
  Point2f cen;
*/
  //Mat drawing = imread("/home/liu/catkin_ws/src/videostream/picrure/left466.jpg");
  Mat drawing = img.clone();
  Mat draw = drawing.clone();
  Mat draw1 = drawing.clone();
  Mat bgr = drawing.clone();
  Mat bgr1 = drawing.clone();

  // Draw contour
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 0 );
       //drawContours( draw, contours, i, color, 1, 8, hierarchy, 0, Point() );
       double area = contourArea(contours[i]);
       cout << "area = " << area << endl;
       if (area > 250) {
         contourrrr.push_back(contours[i]);
         //cout << "contour = " << contours[i] << endl;
         //drawContours( draw, contours, i, color, 1, 8, hierarchy, 0, Point() );
         //cout << "draw contour" << endl;
         //for (int i = 0; i < contours.size(); i++){
         /*
         approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
         minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
         cout << "center of minencircle = " << center[i] << endl;
         cout << "radius of minencircle = " << radius[i] << endl;
         Scalar colors = Scalar( 0, 0, 255 ); //better
         circle( draw, cen, rad, colors, 1, 8, 0 );
         */
       }
     }
/*
  namedWindow("draw", 0);
  imshow("draw", draw);
*/
  //vector<vector<Point> > contours;
  //findContours( drawing, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<vector<Point> > contours_poly( contourrrr.size() );
  vector<Point2f>center( contourrrr.size() );
  vector<float>radius( contourrrr.size() );
  //Point2f center;
  //float radius;
  float rad;
  Point2f cen;

  // Find min enclosing circle
  for( int i = 0; i < contourrrr.size(); i++ )
     { approxPolyDP( Mat(contourrrr[i]), contours_poly[i], 3, true );

       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
       //minEnclosingCircle(contourrrr[i], center, radius);
       cout << "center by minEnclosingCircle = " << center[i] << endl;
       cout << "radius by minEnclosingCircle = " << radius[i] << endl;
       rad = radius[i];
       //cen.push_back(center[i]);
       cen = center[i];
     }
/*
  for( int i = 0; i< contourrrr.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 0 );
       drawContours( draw, contourrrr, i, color, 1, 8, hierarchy, 0, Point() );
     }
*/
/*
  namedWindow("minenclosedcircle", 0);
  imshow("minenclosedcircle", draw);
*/
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
  //vector<Point2f> mc(contourrrr.size());
  Point2f mc;
  for( int i = 0; i< contourrrr.size(); i++ )
    {
      mu[i] = moments(contourrrr[i], false);
    }
  for( int i = 0; i< contourrrr.size(); i++ )
   {
      mc = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
      //cout << mc[i];

      Scalar color = Scalar( 255, 0, 0 );
      circle( draw, mc, rad, color, 1, 8, 0 );
      circle( bgr, mc, 1, color, -1, 8, 0 );

      cout << "center by moment= " << mc << endl;
   }
/*
  for( int i = 0; i< contourrrr.size(); i++ )
     {
       Scalar color = Scalar( 255, 0, 0 ); //worse
       circle( draw, mc[i], rad, color, 1, 8, 0 );
       circle( bgr, mc[i], 1, color, -1, 8, 0 );
       cout << "center of moment= " << mc[i] << endl;
     }*/



   Scalar colors = Scalar( 0, 0, 255 ); //better
   circle( draw, cen, rad, colors, 1, 8, 0 );
   circle( bgr, cen, 1, colors, -1, 8, 0 );

   Scalar color = Scalar( 255, 0, 0 ); //worse
   circle( draw, mc, rad, color, 1, 8, 0 );
   circle( bgr, mc, 1, color, -1, 8, 0 );


  //namedWindow("circle by moment", 0);
  //imshow("circle by moment", draw);
/*
  namedWindow("center of circle by moment", 0);
  imshow("center of circle by moment", bgr);
*/
/*
  namedWindow("circle by moment", 0);
  imshow("circle by moment", draw);

  namedWindow("circle by moment", 0);
  imshow("circle by moment", bgr);
*/
/*
  namedWindow("circle by minEnclosingCircle", 0);
  imshow("circle by minEnclosingCircle", draw1);

  namedWindow("center of circle by minEnclosingCircle", 0);
  imshow("center of circle by minEnclosingCircle", bgr1);
*/

/*
  ROI = draw(Rect(cen.x-200, cen.y-200, 400, 400));
  imshow("ROI", ROI);
*/
  waitKey();

  return 0;
}
