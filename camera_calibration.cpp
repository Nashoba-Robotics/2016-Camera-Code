#include <cv.hpp>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;
//g++ -ggdb `pkg-config opencv --cflags --libs` camera_calibration.cpp -o camera_calib `pkg-config --libs opencv`
int main()
{
  int numBoards = 0;
  int numCornersHor;
  int numCornersVer;
  
  std::cout << "Enter number of corners along width: ";
  std::cin >> numCornersHor;

  std::cout << "Enter number of corners along height: ";
  std::cin >> numCornersVer;

  std::cout << "Enter number of boards: ";
  std::cin >> numBoards;

  int numSquares = numCornersHor * numCornersVer;
  Size board_sz = Size(numCornersHor, numCornersVer);

  VideoCapture capture = VideoCapture(0);
  //capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points;

  vector<Point2f> corners;
  int successes=0;

  Mat image;
  Mat gray_image;

  capture >> image;

  vector<Point3f> obj;
  for(int j=0; j < numSquares; j++)
    obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

  while(successes<numBoards) 
  {
    cvtColor(image, gray_image, CV_BGR2GRAY);
    bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
  
    if(found)
    {
      cornerSubPix(gray_image, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray_image, board_sz, corners, found);
    }
    imshow("win1", image);
    imshow("win2", gray_image);

    capture >> image;
    int key = waitKey(1);
    if(key == 27)
      return 0;

    if(key == ' ' && found != 0)
    {
      image_points.push_back(corners);
      object_points.push_back(obj);
      printf("Snap stored!\n");
      successes++;
      if(successes >= numBoards)
        break;
    }
  }

  Mat intrinsic = Mat(3,3,CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

  intrinsic.ptr<float>(0)[0] = 1;
  intrinsic.ptr<float>(1)[1] = 1;
  calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

    cout << intrinsic << endl;
    cout << distCoeffs << endl;
  

  Mat imageUndistorted;
  while(1)
  {
    capture >> image;
    undistort(image, imageUndistorted, intrinsic, distCoeffs);
    imshow("win1", image);
    imshow("win2", imageUndistorted);
    waitKey(1);
  }
  capture.release();

  return 0;
}
