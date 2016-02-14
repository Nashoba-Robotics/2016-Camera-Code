#include <cv.hpp>
#include <highgui.h>
#include <opencv2/highgui.hpp>

using namespace cv;

int main(int argc, char* argv[])
{
  VideoCapture capture(0);
  Mat img;
  while(true)
  {
    capture >> img;
    imshow("img", img);
    waitKey(1);
  }
  capture.release();
  return 0;
}
