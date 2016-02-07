#include <cv.hpp>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/videoio/videoio_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <stdio.h>
//g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o camera `pkg-config --libs opencv`

int main(int argc, char* argv[])
{
  VideoCapture capture = VideoCapture(0);  
  if(!capture.isOpened())
    return -1;
  cvNamedWindow( "image", 1 );
  cvNamedWindow( "BW Image", 2);
  Mat intrinsic = Mat(3,3,CV_32FC1);
  intrinsic.ptr<float>(0)[0] = 567.3694188707971;
  intrinsic.ptr<float>(0)[1] = 0;
  intrinsic.ptr<float>(0)[2] = 334.050726216;
  intrinsic.ptr<float>(1)[0] = 0;
  intrinsic.ptr<float>(1)[1] = 566.853963425446;
  intrinsic.ptr<float>(1)[2] = 236.5266640528402;
  intrinsic.ptr<float>(2)[0] = 0;
  intrinsic.ptr<float>(2)[1] = 0;
  intrinsic.ptr<float>(2)[2] = 1;
  Mat distCoeffs = Mat(1,5,CV_32FC1);
  distCoeffs.ptr<float>(0)[0] = -0.463731090351821;
  distCoeffs.ptr<float>(0)[1] = 0.5114874359918231;
  distCoeffs.ptr<float>(0)[2] = -0.003444501644466447;
  distCoeffs.ptr<float>(0)[3] = 0.0005056629007096351;
  distCoeffs.ptr<float>(0)[4] = -0.6914960232636986;
  Mat imgFixed;
 
  //capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  Mat img;
  while(1)
  {
    capture >> img;
    undistort(img, imgFixed, intrinsic, distCoeffs);    
    imshow( "image", imgFixed );
    //HLS Threshold stuff
    //HLS = Hue Luminance Saturation
    int H_low = 66;
    int H_high = 96;
    int S_low = 99;
    int S_high = 255;
    int L_low = 41;
    int L_high = 151;
    Mat hls;
    Scalar low = Scalar(H_low, L_low, S_low);
    Scalar high = Scalar(H_high, L_high, S_high);
    Mat imgThresh;
    cvtColor(imgFixed, hls, COLOR_BGR2HLS);
    inRange(hls, low, high, imgThresh);
    imshow( "BW Image", imgThresh);
    
    waitKey(1);
  }
  capture.release();
  img.release();  
  return 0;
}
