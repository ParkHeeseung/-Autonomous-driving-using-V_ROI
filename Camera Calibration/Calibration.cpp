#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include <cv.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

int main(int ac, char** av){

  Mat originImg, grayImg, resultImg;

  VideoCapture capture;
  capture.open(-1);

  int numBoards = 0, numCornersHor, numCornersVer;

  printf("Enter number of corners along width: ");
  scanf("%d", &numCornersHor);

  printf("Enter number of corners along height: ");
  scanf("%d", &numCornersVer);

  printf("Enter number of boards: ");
  scanf("%d", &numBoards);

  int numSquares = numCornersHor * numCornersVer;
  Size board_sz = Size(numCornersHor, numCornersVer);

  vector < vector <Point3f> > object_points;
  vector < vector <Point2f> > image_points;
  vector <Point2f> corners;

  int successes = 0;

  vector<Point3f> obj;
  for(int j=0;j<numSquares;j++)
    obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

  while(successes < numBoards){

    capture >> originImg;

    if (originImg.empty()){
      cerr << "Empty Left Image" << endl;
      return -1;
    }
    cvtColor(originImg, grayImg, CV_BGR2GRAY);

    resultImg = originImg.clone();

    bool found = findChessboardCorners(originImg, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
      cornerSubPix(grayImg, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(resultImg, board_sz, corners, found);

    }

    printf("successes : %d\n", successes);

    imshow("Chessboard", resultImg);

    int key = waitKey(1);

    printf("key : %d\n", key);

    if(key == 27) return 0;

    if(key == ' ' && found != 0){
      image_points.push_back(corners);
      object_points.push_back(obj);

      printf("Snap stored!");

      successes++;

      if(successes >= numBoards) break;
    }

  }

  Mat intrinsic = Mat(3, 3, CV_32FC1); //cameraMatrix
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

  //카메라 비율(가로, 세로)
  intrinsic.ptr<float>(0)[0] = 1;
  intrinsic.ptr<float>(1)[1] = 0.75;

  calibrateCamera(object_points, image_points, originImg.size(), intrinsic, distCoeffs, rvecs, tvecs);

  cout << intrinsic << endl;
  cout << distCoeffs << endl;


  FileStorage fs("camcalib.xml", FileStorage::WRITE);

  fs << "cameraMatrix" << intrinsic;
  fs << "distCoeffs" << distCoeffs;

  fs.release();

  cout << "Write Done." << endl;


  Mat imageUndistorted;

  while(1)
  {
    capture >> originImg;

    undistort(originImg, imageUndistorted, intrinsic, distCoeffs);

    imshow("originImg", originImg);
    imshow("resultImg", imageUndistorted);

    waitKey(1);
  }


  return 0;
}
