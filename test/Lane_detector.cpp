
#include <iostream>
#include <fstream>
#include <ctime>
#include <queue>
#include <cv.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <string.h>
#include <sys/time.h>

using namespace cv;
using namespace std;

const CvScalar COLOR_BLUE = CvScalar(255, 0, 0);
const CvScalar COLOR_RED = CvScalar(0, 0, 255);
const CvScalar COLOR_GREEN = CvScalar(170, 170, 0);

const Vec3b RGB_WHITE_LOWER = Vec3b(100, 100, 150);
const Vec3b RGB_WHITE_UPPER = Vec3b(255, 255, 255);
const Vec3b RGB_YELLOW_LOWER = Vec3b(225, 180, 0);
const Vec3b RGB_YELLOW_UPPER = Vec3b(255, 255, 170);
const Vec3b HSV_YELLOW_LOWER = Vec3b(10, 50, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(40, 140, 255);

const Vec3b HLS_YELLOW_LOWER = Vec3b(20, 120, 80);
const Vec3b HLS_YELLOW_UPPER = Vec3b(45, 200, 255);

const Size imageSize = Size(640, 480);

Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);

void base_ROI(Mat& img, Mat& img_ROI);
void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
float get_slope(const Point& p1, const Point& p2);
bool hough_left(Mat& img, Point* p1, Point* p2);
bool hough_right(Mat& img, Point* p1, Point* p2);
bool hough_curve(Mat& img, Point* p1, Point* p2);
float data_transform(float x, float in_min, float in_max, float out_min, float out_max);
bool get_intersectpoint(const Point& AP1, const Point& AP2,
const Point& BP1, const Point& BP2, Point* IP);
void polyfit(vector <Point> vecP, Point* p1, Point* p2);

int main(){

  //input camera
  VideoCapture capture;
  capture.open("test10.avi");

  if (!capture.isOpened()){
    cerr << "Camera error" << endl;
    return -1;
  }

  //lane detection
  Mat frame, leftROI, rightROI, grayImgL, grayImgR,  cannyImgL, cannyImgR;
  Point p1, p2, p3, p4, p5;

  //curve detection
  Mat oriImg, grayImg;
  Point cp1, cp2;

  //Camera Calibration
  Mat temp, map1, map2;

  //Control var
  float steer, skewness, xLeft, xRight, y = 120, slope, x_Difference;

  //output
  Mat output;


  // FileStorage fs;
  // fs.open("camcalib.xml", FileStorage::READ);
  //
  // if(!fs.isOpened()){
  //   cerr << "Failed to opend"<< endl;
  //   return -1;
  // }
  //
  // fs["cameraMatrix"] >> cameraMatrix;
  // fs["distCoeffs"] >> distCoeffs;
  //
  // fs.release();
  //
  // initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);

  bool left_error = true, right_error = true, curve_error = true;

  while(1){

    capture >> frame;
    // temp = frame.clone();
    // remap(frame, temp, map1, map2, CV_INTER_LINEAR);

    leftROI = frame(Rect(0, frame.rows/4 * 3, frame.cols/2, frame.rows/4));
    rightROI = frame(Rect(frame.cols/2, frame.rows/4 * 3, frame.cols/2, frame.rows/4));

    cvtColor(leftROI, grayImgL, CV_BGR2GRAY);
    cvtColor(rightROI, grayImgR, CV_BGR2GRAY);

    Canny(grayImgL, cannyImgL, 150, 270);
    Canny(grayImgR, cannyImgR, 150, 270);

    printf("hi");
    left_error = hough_left(cannyImgL, &p1, &p2);
    right_error = hough_right(cannyImgR, &p3, &p4);


    if(left_error || right_error){

      oriImg = frame(Rect(0, frame.rows/4 * 3, frame.cols, frame.rows/4));

      cvtColor(oriImg, grayImg, CV_BGR2GRAY);

      Canny(grayImg, cannyImgL, 150, 270);

      curve_error = hough_curve(cannyImgL, &cp1, &cp2);

      slope = get_slope(cp1, cp2);



      if(curve_error){
        steer = 0.0;
      }
      else if(slope < 0.0){
      //right rotate (0.0 ~ 1.0)
        steer =  data_transform(slope, -1.2, -0.2, 0.0, 1.0);
        xLeft = (y - cp1.y + slope * cp1.x) / slope;

        skewness = data_transform(xLeft, -200.0, 50.0, 0.0, 1.0);
        steer = -(steer * skewness);
      }
      else{
      //left rotate (-1.0 ~ 0.0)
        steer =  data_transform(slope, 0.2, 1.2, -1.0, 0.0);
        xRight = (y - cp1.y + slope * cp1.x) / slope;

        skewness = data_transform(xRight, 600, 800, -1.0, 0.0);
        steer = (steer * skewness);

      }

      line(oriImg, cp1, cp2, COLOR_RED, 4, CV_AA);

      imshow("curve_output", oriImg);

    }
    else{
      line(leftROI, p1, p2, COLOR_RED, 4, CV_AA);
      line(rightROI, p3, p4, COLOR_RED, 4, CV_AA);

      hconcat(leftROI, rightROI, output);


      get_intersectpoint(p1, p2, Point(p3.x + 320, p3.y), Point(p4.x + 320, p4.y), &p5);

      x_Difference = 200.0 - p5.x;

      if(x_Difference > 0.0){
        steer = data_transform(x_Difference, 0.0, 200.0, 0.0, 0.3);
        steer = -steer;
      }
      else if(x_Difference < 0.0){
        steer = data_transform(x_Difference, -440.0, 0.0, -0.3, 0.0);
        steer = -steer;
      }
      else{
        steer = 0.0;
      }

      imshow("reuslt", output);
    }


    if(steer > 1.0){
      steer = 1.0;
    }
    else if(steer < -1.0){
      steer = -1.0;
    }

    printf("%lf\n", steer);

    //pipe file
    // FILE* fp = fopen("/home/nvidia/Desktop/p2", "w");
    // fprintf(fp, "%f", steer);
    // fclose(fp);

    waitKey(0);

    if (waitKey(10) == 0) {
      return 0;
    }

  }
}

void base_ROI(Mat& img, Mat& img_ROI) {

	Point a = Point(0, 40);
	Point b = Point(0, img.rows);
	Point c = Point(img.cols, img.rows);
	Point d = Point(img.cols, 40);

	vector <Point> Left_Point;

	Left_Point.push_back(a);
	Left_Point.push_back(b);
	Left_Point.push_back(c);
	Left_Point.push_back(d);

	Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

	fillConvexPoly(roi, Left_Point, Scalar(255));

	Mat filteredImg_Left;
	img.copyTo(filteredImg_Left, roi);

	img_ROI = filteredImg_Left.clone();

}

void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2){

  float slope = get_slope(p1, p2);
  float alphaY = 40.f / sqrt(slope*slope + 1);
  float alphaX = slope * alphaY;

  Point a(p1.x - alphaX, p1.y + alphaY );
  Point b(p1.x + alphaX, p1.y - alphaY );
  Point c(p2.x + alphaX, p2.y - alphaY );
  Point d(p2.x - alphaX, p2.y + alphaY );

  vector <Point> Left_Point;

  Left_Point.push_back(a);
  Left_Point.push_back(b);
  Left_Point.push_back(c);
  Left_Point.push_back(d);

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);


  img_ROI = filteredImg_Left.clone();

}


float get_slope(const Point& p1, const Point& p2){

  float slope;

  slope = ((float) p2.y - (float) p1.y) / ((float) p2.x - (float) p1.x);

  return slope;
}

bool get_intersectpoint(const Point& AP1, const Point& AP2,
                       const Point& BP1, const Point& BP2, Point* IP)
{
    double t;
    double s;
    double under = (BP2.y-BP1.y)*(AP2.x-AP1.x)-(BP2.x-BP1.x)*(AP2.y-AP1.y);
    if(under==0) return false;

    double _t = (BP2.x-BP1.x)*(AP1.y-BP1.y) - (BP2.y-BP1.y)*(AP1.x-BP1.x);
    double _s = (AP2.x-AP1.x)*(AP1.y-BP1.y) - (AP2.y-AP1.y)*(AP1.x-BP1.x);

    t = _t/under;
    s = _s/under;

    if(t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
    if(_t==0 && _s==0) return false;

    IP->x = AP1.x + t * (double)(AP2.x-AP1.x);
    IP->y = AP1.y + t * (double)(AP2.y-AP1.y);

    return true;
}

bool hough_left(Mat& img, Point* p1, Point* p2) {

	vector<Vec2f> linesL;
  vector<Vec2f> newLinesL;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){

    HoughLines(img, linesL, 1, CV_PI / 180, threshold);

    for(size_t j = 0; j < linesL.size(); j++){

      Vec2f temp;

      float rho = linesL[j][0];
      float theta = linesL[j][1];

      if(CV_PI / 18 >= theta || theta >= CV_PI / 18 * 8) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesL.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesL.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesL.size() > 1) {
  			for (size_t i = 0; i < newLinesL.size(); i++) {
  				count++;
  				float rho = newLinesL[i][0];
  				float theta = newLinesL[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;


  		return false;
  	}
  	return true;
}

bool hough_right(Mat& img, Point* p1, Point* p2) {

	vector<Vec2f> linesR;
  vector<Vec2f> newLinesR;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesR, 1, CV_PI / 180, threshold);



    for(size_t j = 0; j < linesR.size(); j++){

      Vec2f temp;

      float rho = linesR[j][0];
      float theta = linesR[j][1];

      if(CV_PI / 18 * 10 >= theta || theta >= CV_PI / 18 * 17) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesR.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesR.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesR.size() > 1) {
  			for (size_t i = 0; i < newLinesR.size(); i++) {
  				count++;
  				float rho = newLinesR[i][0];
  				float theta = newLinesR[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;

  		return false;
  	}
  	return true;
}


bool hough_curve(Mat& img, Point* p1, Point* p2) {

	vector<Vec2f> lines;

  Point PointAll;
  Point a, b;


  vector <Point> vecP;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){
    HoughLines(img, lines, 1, CV_PI / 180, threshold);

  	if (lines.size() > 1) {
  		for (size_t i = 0; i < lines.size(); i++) {
  			count++;
  			float rho = lines[i][0];
  			float theta = lines[i][1];


  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

        PointAll.x = x0;
        PointAll.y = y0;

        vecP.push_back(PointAll);

  		}

      polyfit(vecP, &a, &b);

  		break;

  		};
  	}
  	if (count != 0) {

      p1->x = a.x;
      p2->x = b.x;
      p1->y = a.y;
      p2->y = b.y;

  		return false;
  	}
  	return true;
}

float data_transform(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void polyfit(vector <Point> vecP, Point* p1, Point* p2)
{
    int i,j,k,n,N;

    N = vecP.size();

    double x[N],y[N];
    cout<<"\nEnter the x-axis values:\n";                //Input x-values
    for (i=0;i<N;i++)
        x[i] = vecP.at(i).x;
    cout<<"\nEnter the y-axis values:\n";                //Input y-values
    for (i=0;i<N;i++)
        y[i] = vecP.at(i).y;

    cout<<"\nWhat degree of Polynomial do you want to use for the fit?\n";
    n = 1;

                               // n is the degree of Polynomial
    double X[2*n+1];
                         //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i=0;i<2*n+1;i++)
    {
        X[i]=0;
        for (j=0;j<N;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<n+1;i++)
    {
        Y[i]=0;
        for (j=0;j<N;j++)
        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i=0;i<=n;i++)
        B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";
    for (i=0;i<n;i++)            //print the Normal-augmented matrix
    {
        for (j=0;j<=n;j++)
            cout<<B[i][j]<<setw(16);
        cout<<"\n";
    }
    for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=n;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }

    for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n;k++)
            {
                double t=B[k][i]/B[i][i];
                for (j=0;j<=n;j++)
                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
    for (i=n-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }

    cout<<"\nThe values of the coefficients are as follows:\n";
    for (i=0;i<n;i++)

        cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....



    p1->y = 0;
    p2->y = 480;

    float p1_x = (0.0 - (a[1])) / a[0];
    float p2_x = (480.0 - (a[1])) / a[0];

    p1->x = p1_x;
    p2->x = p2_x;

    cout << "p1_X : " << p1->x << "p1_Y : " << p1->y << endl;
    cout << "p2_X : " << p2->x << "p2_Y : " << p2->y << endl;

    return;
  }
