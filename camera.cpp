#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <math.h>
#include "GetImage.h"

#define ShowWindows

//#define r640x480
#define r1280x720
//#define r1920x1080

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

//g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o camera `pkg-config --libs opencv`

using namespace cv;
using namespace std;

GetImage getimg;

Mat getBWImage( Mat intrinsic, Mat distCoeffs) {
  //Dilation
  const int dilationSize = 2;
  const Mat dilateElement = getStructuringElement(MORPH_RECT, Size(2*dilationSize + 1, 2*dilationSize + 1), Point(dilationSize, dilationSize));

  //Blur
  const int kernelSize = 8*1+ 1;

  //HLS Thresholding
  const int H_low = 60;
  const int H_high = 180;
  const int S_high = 255;
  const int S_low = 78;
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
  
	//cap >> img;
  img = getimg.mainloop();
  //Undistortion processing
  undistort(img, imgFixed, intrinsic, distCoeffs);
#ifdef ShowWindows
  imshow( "image", imgFixed );
#endif
  //Blur
  GaussianBlur(imgFixed,blurredImg,Size(kernelSize,kernelSize), 1);
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

Mat getIntrinsic() {
  //Undistortion constants
  Mat intrinsic = Mat(3,3,CV_32FC1);
#ifdef r640x480
  intrinsic.ptr<float>(0)[0] = 567.3694188707971;
  intrinsic.ptr<float>(0)[1] = 0;
  intrinsic.ptr<float>(0)[2] = 334.050726216;
  intrinsic.ptr<float>(1)[0] = 0;
  intrinsic.ptr<float>(1)[1] = 566.853963425446;
  intrinsic.ptr<float>(1)[2] = 236.5266640528402;
  intrinsic.ptr<float>(2)[0] = 0;
  intrinsic.ptr<float>(2)[1] = 0;
  intrinsic.ptr<float>(2)[2] = 1;
#elif defined(r1280x720)
  //These should be done again
  intrinsic.ptr<float>(0)[0] = 849.256399173029;
  intrinsic.ptr<float>(0)[1] = 0;
  intrinsic.ptr<float>(0)[2] = 655.2627383935369;
  intrinsic.ptr<float>(1)[0] = 0;
  intrinsic.ptr<float>(1)[1] = 850.2670498542308;
  intrinsic.ptr<float>(1)[2] = 361.233269254571;
  intrinsic.ptr<float>(2)[0] = 0;
  intrinsic.ptr<float>(2)[1] = 0;
  intrinsic.ptr<float>(2)[2] = 1;
#endif

  return intrinsic;
}

Mat getDistCoeffs() {
  Mat distCoeffs = Mat(1,5,CV_32FC1);
#ifdef r640x480
  distCoeffs.ptr<float>(0)[0] = -0.463731090351821;
  distCoeffs.ptr<float>(0)[1] = 0.5114874359918231;
  distCoeffs.ptr<float>(0)[2] = -0.003444501644466447;
  distCoeffs.ptr<float>(0)[3] = 0.0005056629007096351;
  distCoeffs.ptr<float>(0)[4] = -0.6914960232636986;
#elif defined(r1280x720)
  //These should be done again
  distCoeffs.ptr<float>(0)[0] = -0.1019179702473763;
  distCoeffs.ptr<float>(0)[1] = -0.09943304239604193;
  distCoeffs.ptr<float>(0)[2] = 0.003811821061129521;
  distCoeffs.ptr<float>(0)[3] = 0.002936280291171185;
  distCoeffs.ptr<float>(0)[4] = 0.1175824546873729;
#endif

  return distCoeffs;
}

int main(int argc, char* argv[])
{
  cout << "Using OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;
/* 
 VideoCapture capture = VideoCapture(0);  
  if(!capture.isOpened()) {
    cout << "Video Capture not opened" << endl;
    return -1;
  } 
 */

  getimg = GetImage();
  getimg.open_device();
  getimg.init_device();
  getimg.start_capturing();
 
  //Contours
  const int minArea = 2000;
  const int thresh = 200; //For edge detection 
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  const Scalar color = Scalar(255,255,255);

  //Angle and distance detection
  const int z = 4;
  const int h_naught = 12;
  const double f = 686;
  const int w_naught = 20;
  

  while(1)
  {
    clock_t t = clock();
    Mat dilatedImg;
    dilatedImg = getBWImage( getIntrinsic(), getDistCoeffs());
    //Contours processing
    Canny(dilatedImg, canny_output, thresh, thresh*2, 3 );
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
        cout << "Adding one to good rect" << endl;
        cout << "\t Width : " << boundRect[i].width << endl;
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
    cout << "Good Rect size: " << goodRect.size() << endl; 
    //Determine which contour to use.
    //We'll assume there's only one contour for now, but this needs to be fixed.
    for(unsigned int i = 0; i < goodRect.size(); i++)
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
    cout << CLOCKS_PER_SEC/(clock() - t) << "fps" << endl;
    waitKey(1);
  }
  //capture.release();
  getimg.stop_capturing();
  getimg.uninit_device();
  getimg.close_device();
  return 0;
}
