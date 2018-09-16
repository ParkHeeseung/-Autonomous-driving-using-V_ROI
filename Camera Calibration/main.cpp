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

const Size imageSize = Size(640, 480);

Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);

static void help(char** av){
  cout << endl
    << av[0] << " shows the usage of the OpenCV serialization functionality."         << endl
    << "usage: "                                                                      << endl
    <<  av[0] << " outputfile.yml.gz"                                                 << endl
    << "The output file may be either XML (xml) or YAML (yml/yaml). You can even compress it by "
    << "specifying this in its extension like xml.gz yaml.gz etc... "                  << endl
    << "With FileStorage you can serialize objects in OpenCV by using the << and >> operators" << endl
    << "For example: - create a class and have it serialized"                         << endl
    << "             - use it to read and write matrices."                            << endl;
}

class MyData{
  public:
    MyData() : A(0), X(0), id(){}
    explicit MyData(int) : A(97), X(CV_PI), id("mydata1234"){} // explicit to avoid implicit conversion
    void write(FileStorage& fs) const                        //Write serialization for this class
    {
      fs << "{" << "A" << A << "X" << X << "id" << id << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
      A = (int)node["A"];
      X = (double)node["X"];
      id = (string)node["id"];
    }
  public:   // Data Members
    int A;
    double X;
    string id;
  };

  //These write and read functions must be defined for the serialization in FileStorage to work
static void write(FileStorage& fs, const std::string&, const MyData& x){
  x.write(fs);
}
static void read(const FileNode& node, MyData& x, const MyData& default_value = MyData()){
  if(node.empty())  x = default_value;
  else  x.read(node);
}

int main(int ac, char *av[]){

  Mat originImg, grayImg, resultImg, frame, temp, map1, map2;


  FileStorage fs;
  fs.open("camcalib.xml", FileStorage::READ);

  if(!fs.isOpened()){
    cerr << "Failed to opend"<< endl;
    return -1;
  }

  fs["cameraMatrix"] >> cameraMatrix;
  fs["distCoeffs"] >> distCoeffs;

  fs.release();

  VideoCapture capture;
  capture.open(-1);

  if (!capture.isOpened()){
    cerr << "Camera error" << endl;
    return -1;
  }

  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);

  while(1){

    capture >> frame;
    temp = frame.clone();
    remap(frame, temp, map1, map2, CV_INTER_LINEAR);

    imshow("originImg", frame);
    imshow("Test", temp);

    waitKey(1);

  }


  return 0;
}
