#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <math.h>
#include "GetImage.h"
#include <errno.h>
#include <wiringPiSPI.h>

//#define ShowWindows

#define r640x480
//#define r1280x720
//#define r1920x1080

//#define Blur

#ifdef r1280x1720
#define WIDTH 1280
#define HEIGHT 720
#elif defined(r640x480)
#define WIDTH 640
#define HEIGHT 480
#elif defined(r1920x1080)
#define WIDTH 1920
#define HEIGHT 1080
#endif

#define USE_SPI
#define SPI_CHANNEL 0
#define SPI_SPEED 500000

#define HUE_LOW 60
#define HUE_HIGH 180
#define LUM_LOW 100
#define LUM_HIGH 255
#define SAT_LOW 28
#define SAT_HIGH 255

using namespace cv;
using namespace std;

VideoCapture capture;

Mat getBWImage() {
  //Dilation
  const int dilationSize = 2;
  const Mat dilateElement = getStructuringElement(MORPH_RECT, Size(2*dilationSize + 1, 2*dilationSize + 1), Point(dilationSize, dilationSize));

#ifdef Blur
  //Blur
  const int kernelSize = 8*1+ 1;
#endif

  //HLS Thresholding
  const Scalar low = Scalar(HUE_LOW, LUM_LOW, SAT_LOW);
  const Scalar high = Scalar(HUE_HIGH, LUM_HIGH, SAT_HIGH);

  Mat img;
  Mat imgFixed;
  Mat blurredImg;
  Mat imgThresh;
  Mat dilatedImg;
  Mat hls;
  
  capture >> img;

#ifdef ShowWindows
  imshow( "image", img);
#endif

  //Blur
#ifdef Blur
  GaussianBlur(img,blurredImg,Size(kernelSize,kernelSize), 1);
#else
  blurredImg = img;
#endif

  //HLS Threshold processing
  cvtColor(blurredImg, hls, COLOR_BGR2HLS);
  inRange(hls, low, high, imgThresh);
  //Dilation
  dilate(imgThresh, dilatedImg,dilateElement); 
#ifdef ShowWindows
  imshow("dilate", dilatedImg);
#endif 
 return dilatedImg;
}

int main(int argc, char* argv[])
{
  setNumThreads(4);

#ifdef USE_SPI
  cout << "Initializing wiringPi" << endl;

  int spiID = wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);

  if(spiID == -1) {
    int err = errno;
    cout << "WiringPi SPI initialization failed: " << strerror(err) << endl;
  }
#endif

  cout << "Using OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;
  capture = VideoCapture(0);  
  if(!capture.isOpened()) {
    cout << "Video Capture not opened" << endl;
    return -1;
  } 

  //Contours
  const int minArea = 2000;
  const int thresh = 200; //For edge detection 
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  const Scalar color = Scalar(255,255,255);

  while(1)
  {
    clock_t t = clock();
    Mat img = getBWImage();
    //Contours processing
    Canny(img, canny_output, thresh, thresh*2, 3 );
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    //Limit contours (area, perimeter, etc.)
    vector<vector<Point> > goodContours(0);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(0);
    vector<Rect> goodRect(0);
    for(unsigned int i = 0; i < contours.size(); i++)
    {
      approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
      boundRect.push_back(boundingRect(Mat(contours_poly[i])));
      if(boundRect[i].width * boundRect[i].height > minArea){
        goodContours.push_back(contours[i]);
        goodRect.push_back(boundRect[i]);
      }
    }
    //Draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3 );
    for(unsigned int i = 0; i < goodContours.size(); i++)
    {
      rectangle(drawing, goodRect[i].tl(), goodRect[i].br(), color, 2,8,0);
      drawContours(drawing, goodContours, i, color, 2,8,hierarchy, 0, Point() );
    }
#ifdef ShowWindows
    imshow("Contours", drawing);
#endif
    //Determine which contour to use.
    //We'll assume there's only one contour for now, but this needs to be fixed.
    for(unsigned int i = 0; i < goodRect.size(); i++)
    {
      if(goodRect.size() > 0) {
        cout << "New image: " << i << endl;
        cout << "height: " << goodRect[i].height << endl;
        cout << "width: \t" << goodRect[i].width << endl;

        //TODO: These are placeholder variables for testing.
        //      We need to find the actual distance and angle here.
        //      These are in units of 16ths of an inch and hundreths of degrees 
        int distance = goodRect[i].height;
        int angleToTurn = goodRect[i].width;

        cout << "Distance: \t" << distance << endl;
        cout << "Angle: \t\t" << angleToTurn << endl;

#ifdef USE_SPI
        unsigned char data[8] = {0};
        data[0] = distance;
        data[4] = angleToTurn;

        int result = wiringPiSPIDataRW(SPI_CHANNEL, data, 8);
        
        if(result < 0) {
          int err = errno;
          cout << "SPI had an error: " << strerror(err) << endl;
        }
#endif

      }
    }
    cout << CLOCKS_PER_SEC/(clock() - t) << "fps" << endl;
    //waitKey(1);
  }
  capture.release();
  return 0;
}
