#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/videoio/videoio_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <stdio.h>
//g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o camera `pkg-config --libs opencv`

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
  VideoCapture capture = VideoCapture(0);  
  if(!capture.isOpened())
    return -1;
  cvNamedWindow( "image", 1 );
  cvNamedWindow( "BW Image", 2);
  //Undistortion constants
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

  //HLS Threshold
  //HLS = Hue Luminance Saturation
  int H_low = 0;
  int H_high = 29;
  int S_low = 0;
  int S_high = 47;
  int L_low = 218;
  int L_high = 255;
  Mat hls;
  Scalar low = Scalar(H_low, L_low, S_low);
  Scalar high = Scalar(H_high, L_high, S_high);
  Mat imgThresh;
 
  //Contours
  int minArea = 10000;
  int minPerimeter;
  int minWidth;
  int minHeight;
  int maxWidth;
  int maxHeight;
  int minSolidity;
  int maxSolidity;
  int thresh = 255; //For edge detection 
  RNG rng(12345);  

  //capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  Mat img;
  while(1)
  {
    capture >> img;
    //Undistortion processing
    undistort(img, imgFixed, intrinsic, distCoeffs);    
    imshow( "image", imgFixed );
    //Blur
    Mat blurredImg;
    int kernelSize = 8*1+ 1;
    GaussianBlur(imgFixed,blurredImg,Size(kernelSize,kernelSize), 1); 
    //HLS Threshold processing
    cvtColor(blurredImg, hls, COLOR_BGR2HLS);
    inRange(hls, low, high, imgThresh);
    imshow( "BW Image", imgThresh);
    //Dilation
    int dilationSize = 2;
    Mat dilatedImg;
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(2*dilationSize + 1, 2*dilationSize + 1), Point(dilationSize, dilationSize));
    dilate(imgThresh, dilatedImg,dilateElement);
    imshow( "Dilated Image", dilatedImg);
    //Contours processing
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Canny(dilatedImg, canny_output, thresh, thresh*2, 3 );
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    //Limit contours (area, perimeter, etc.)
    vector<vector<Point> > goodContours(contours.size());
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Rect> goodRect(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
      approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
      boundRect[i] = boundingRect(Mat(contours_poly[i]));
      if(boundRect[i].width * boundRect[i].height > minArea){
        goodContours.push_back(contours[i]);
        goodRect.push_back(boundRect[i]);
      }
    }
    
    //Draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3 );
    //Scalar color = Scalar(255,255,255);
    for(int i = 0; i < goodContours.size(); i++)
    {
      Scalar color = Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
      rectangle(drawing, goodRect[i].tl(), goodRect[i].br(), color, 2,8,0);
      drawContours(drawing, goodContours, i, color, 2,8,hierarchy, 0, Point() );
    }
    imshow("Contours", drawing);
    waitKey(1);
  }
  capture.release();
  img.release();  
  return 0;
}
