
#include "opencv2/opencv.hpp"
 
using namespace cv;
 
int main(int argc, char** argv)
{
	// ******* No Transparent API(UMat) *******
	double mat_start  = static_cast<double>(getTickCount());
	Mat img, gray;
	img = imread("/home/lab606a/Pictures/Screenshot 2020-02-20 14:15:18.png", IMREAD_COLOR);
 
	cvtColor(img, gray, CV_BGR2GRAY);
	GaussianBlur(gray, gray, Size(7,7), 1.5);
	Canny(gray, gray, 0, 50);
 
	std::cout << " Mat costs time: "<< static_cast<double>((getTickCount() - mat_start) / getTickFrequency()) << " s..."<< std::endl;

	imshow("edges", gray);
	//waitKey(0);
 
	
	// ******* Transparent API(UMat) *******
	double umat_start = static_cast<double>(getTickCount());
	UMat uimg, ugray;
	imread("/home/lab606a/Pictures/Screenshot 2020-02-20 14:15:18.png", IMREAD_COLOR).copyTo(uimg);
 
	cvtColor(uimg, ugray, COLOR_BGR2GRAY);
	GaussianBlur(ugray, ugray, Size(7,7), 1.5);
	Canny(ugray, ugray, 0, 50);
 
	std::cout << "UMat costs time: " << static_cast<double>((getTickCount() - umat_start) / getTickFrequency()) << " s..." << std::endl;
 
	imshow("edges_UMat", ugray);
	waitKey(0);
 
	return 0;
}
