#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/gpu/gpu.hpp>
#include <math.h>
#include "GetImage.h"

#define ShowWindows

#define SmartCapture

//#define r640x480
//#define r1280x720
#define r1920x1080

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
using namespace gpu;

#ifndef SmartCapture
VideoCapture capture;
#else
GetImage getimg;
#endif

Mat intrinsic;
Mat distCoeffs;

void setIntrinsic() {
  //Undistortion constants
  intrinsic = Mat(3,3,CV_32FC1);
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
#elif defined(r1920x1080)
  //These should be done again
  intrinsic.ptr<float>(0)[0] = 1223.416576031427;
  intrinsic.ptr<float>(0)[1] = 0;
  intrinsic.ptr<float>(0)[2] = 968.8462139173206;
  intrinsic.ptr<float>(1)[0] = 0;
  intrinsic.ptr<float>(1)[1] = 1206.547540238048;
  intrinsic.ptr<float>(1)[2] = 539.136613717971;
  intrinsic.ptr<float>(2)[0] = 0;
  intrinsic.ptr<float>(2)[1] = 0;
  intrinsic.ptr<float>(2)[2] = 1;
#endif
}

void setDistCoeffs() {
  distCoeffs = Mat(1,5,CV_32FC1);
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
#elif defined(r1920x1080)
  //These should be done again
  distCoeffs.ptr<float>(0)[0] = -0.2689808925627709;
  distCoeffs.ptr<float>(0)[1] = 0.2881675340793562;
  distCoeffs.ptr<float>(0)[2] = -0.01190096876276858;
  distCoeffs.ptr<float>(0)[3] = -0.02486191381892134;
  distCoeffs.ptr<float>(0)[4] = -0.1603926525077735;
#endif
}


GpuMat getBWImage() {
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
  GpuMat blurredImg;
  Mat imgThresh;
  Mat dilatedImg;
  GpuMat hls;

#ifdef SmartCapture
  img = getimg.mainloop();
#else
  capture >> img;
#endif
  //Undistortion processing
  undistort(img, imgFixed, intrinsic, distCoeffs);
  GpuMat imgFixedGpu(imgFixed);
#ifdef ShowWindows
//  imshow( "image", imgFixed );
#endif
  //Blur
  gpu::GaussianBlur(imgFixedGpu,blurredImg,Size(kernelSize,kernelSize), 1);
  //HLS Threshold processing
  gpu::cvtColor(blurredImg, hls, COLOR_BGR2HLS);
  inRange(Mat(hls), low, high, imgThresh);
  //Dilation
  dilate(imgThresh, dilatedImg,dilateElement); 
#ifdef ShowWindows
  imshow("dilate", dilatedImg);
#endif 
  return GpuMat(dilatedImg);
}

int main(int argc, char* argv[])
{
  cout << "Using OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;

  int numGPU = getCudaEnabledDeviceCount();
  cout << "Number of GPU Devices: " << numGPU << endl;
  
  if(numGPU == 0) {
    cout << "Need to have a GPU Device" << endl;
    return 1;
  }
  
  setDevice(0);

  setDistCoeffs();
  setIntrinsic();

#ifdef SmartCapture
  getimg = GetImage();
  getimg.open_device();
  getimg.init_device();
  getimg.start_capturing();
#else
  capture = VideoCapture(0);
#endif
  //Contours
  const int minArea = 2000;
  const int thresh = 200; //For edge detection 
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  const Scalar color = Scalar(255,255,255);

  //Angle and distance detection
  const int z = -3.25;
  const int h_naught = 12;
#ifdef r640x480
  const double f = 686;
#elif defined r1280x720
  const double f = 1330.5;
#elif defined r1920x1080
  const double f = 1330.5;
#endif
  const int w_naught = 20;
  

  while(1)
  {
    clock_t t = clock();
    GpuMat dilatedImg = getBWImage();
    //Contours processing
    GpuMat canny_output;
    gpu::Canny(dilatedImg, canny_output, thresh, thresh*2, 3 );
    findContours( Mat(canny_output), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
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
        const double turn = asin((w_naught * ((WIDTH/2) - goodRect[i].x))/(goodRect[i].width * d));
         
        cout << "New image: " << i << endl;
        cout << "\tAlpha: \t" << alpha << endl;
        cout << "\td: \t" << d << endl;
        cout << "\tpos turn means we need to turn counter clockwise" << endl;
        cout << "\tturn: \t" << turn << endl;
        cout << "\theight: " << goodRect[i].height << endl;
        cout << "\twidth: \t" << goodRect[i].width << endl;
      }
    }
    cout << CLOCKS_PER_SEC*1.0/(clock() - t) << "fps" << endl;
    waitKey(1);
  }
#ifndef SmartCapture
  capture.release();
#else
  getimg.stop_capturing();
  getimg.uninit_device();
  getimg.close_device();
#endif  
  return 0;
}
