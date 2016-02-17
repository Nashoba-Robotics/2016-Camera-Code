#include <cv.hpp>
#include <highgui.h>
#include <opencv2/highgui.hpp>
#include "GetImage.h"

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

#define SmartCapture

using namespace cv;

int main(int argc, char* argv[])
{
#ifndef SmartCapture
  VideoCapture capture();
#else
  GetImage getimg = GetImage();
  getimg.open_device();
  getimg.init_device();
  getimg.start_capturing();
#endif
  Mat img;
  clock_t t;
  while(true)
  {
   t = clock();
#ifdef SmartCapture
    img = getimg.mainloop();
#else
    capture >> img;
#endif
    imshow("img", img);
    waitKey(1);
  
   fprintf(stderr, "fps: %lf\n",1/((clock() - t)*1.0/CLOCKS_PER_SEC));
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
