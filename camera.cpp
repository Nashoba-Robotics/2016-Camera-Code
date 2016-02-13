#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <math.h>
#include <math.h>
//g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o camera `pkg-config --libs opencv`

using namespace cv;
using namespace std;

Mat getBWImage(VideoCapture cap, Mat intrinsic, Mat distCoeffs) {
  //Dilation
  const int dilationSize = 2;
  const Mat dilateElement = getStructuringElement(MORPH_RECT, Size(2*dilationSize + 1, 2*dilationSize + 1), Point(dilationSize, dilationSize));

  //Blur
  const int kernelSize = 8*1+ 1;

  //HLS Thresholding
  const int H_low = 60;
  const int H_high = 180;
  const int S_low = 78;
  const int S_high = 255;
  const int L_low = 100;
  const int L_high = 255;
  const Scalar low = Scalar(H_low, L_low, S_low);
  const Scalar high = Scalar(H_high, L_high, S_high);

  Mat img;
  Mat imgFixed;
  Mat blurredImg;
  Mat imgThresh;
  Mat dilatedImg;
  Mat hls;
  cap >> img;
  //Undistortion processing
  undistort(img, imgFixed, intrinsic, distCoeffs);
  imshow( "image", imgFixed );
  //Blur
  GaussianBlur(imgFixed,blurredImg,Size(kernelSize,kernelSize), 1);
  //HLS Threshold processing
  cvtColor(blurredImg, hls, COLOR_BGR2HLS);
  inRange(hls, low, high, imgThresh);
  //Dilation
  dilate(imgThresh, dilatedImg,dilateElement);
  imshow("dilate", dilatedImg);
  return dilatedImg;
}

Mat getIntrinsic() {
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
  return intrinsic;
}

Mat getDistCoeffs() {
  Mat distCoeffs = Mat(1,5,CV_32FC1);
  distCoeffs.ptr<float>(0)[0] = -0.463731090351821;
  distCoeffs.ptr<float>(0)[1] = 0.5114874359918231;
  distCoeffs.ptr<float>(0)[2] = -0.003444501644466447;
  distCoeffs.ptr<float>(0)[3] = 0.0005056629007096351;
  distCoeffs.ptr<float>(0)[4] = -0.6914960232636986;
  return distCoeffs;
}

int main(int argc, char* argv[])
{
  cout << CV_MAJOR_VERSION << endl;
  VideoCapture capture = VideoCapture(0);  
  if(!capture.isOpened()) {
    cout << "Video Capture not opened" << endl;
    return -1;
  } 
  
  //Contours
  const int minArea = 500;
  int minPerimeter;
  int minWidth;
  int minHeight;
  int maxWidth;
  int maxHeight;
  int minSolidity;
  int maxSolidity;
  const int thresh = 255; //For edge detection 
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  const Scalar color = Scalar(255,255,255);

  //Angle and distance detection
  const double pi = 3.14159265;
  const int z = 6;
  const int h_naught = 12;
  const double f = 613;
  const int w_naught = 20;
  
  //capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  while(1)
  {
    Mat dilatedImg;
    dilatedImg = getBWImage(capture, getIntrinsic(), getDistCoeffs());
    //Contours processing
    Canny(dilatedImg, canny_output, thresh, thresh*2, 3 );
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    //Limit contours (area, perimeter, etc.)
    vector<vector<Point> > goodContours(0);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(0);
    vector<Rect> goodRect(0);
    for(int i = 0; i < contours.size(); i++)
    {
      approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
      boundRect.push_back(boundingRect(Mat(contours_poly[i])));
      if(boundRect[i].width * boundRect[i].height > minArea){
        cout << "Adding one to good rect" << endl;
        cout << "\t Width : " << boundRect[i].width << endl;
        goodContours.push_back(contours[i]);
        goodRect.push_back(boundRect[i]);
      }
    }
    //Draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3 );
    for(int i = 0; i < goodContours.size(); i++)
    {
      rectangle(drawing, goodRect[i].tl(), goodRect[i].br(), color, 2,8,0);
      drawContours(drawing, goodContours, i, color, 2,8,hierarchy, 0, Point() );
    }
    imshow("Contours", drawing);
    cout << "Good Rect size: " << goodRect.size() << endl; 
    //Determine which contour to use.
    //We'll assume there's only one contour for now, but this needs to be fixed.
    for(int i = 0; i < goodRect.size(); i++)
    {
      if(goodRect.size() > 0) {
        const double alpha = 0.5*(asin(2*z*goodRect[i].height / (f*h_naught)));
        const double d = z/sin(alpha);
        const double beta = acos(d * goodRect[i].width / (f * w_naught));
        const double beta_h = asin(sin(beta) / cos(alpha));
         
        cout << "New image: " << i << endl;
        cout << "Alpha: \t" << alpha << endl;
        cout << "d: \t" << d << endl;
        cout << "beta: \t" << beta << endl;
        cout << "beta_h: " << beta_h << endl;
        cout << "height: " << goodRect[i].height << endl;
        cout << "width: \t" << goodRect[i].width << endl;
      }
    }
    waitKey(1);
  }
  capture.release();
  return 0;
}
