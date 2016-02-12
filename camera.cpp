#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <math.h>
#include <math.h>
#include <ctime>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <libexplain/ioctl.h>
//g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o camera `pkg-config --libs opencv`

bool camActive = false;
fd_set fds;

using namespace cv;
using namespace std;

static int xioctl(int fd, unsigned long request, void *arg)
{
  int r;
  do {
    r = ioctl(fd,request,arg);
    cout << "Test: Request: " << request << " Errno: " << errno << endl;
    cout << "Explanation: " << explain_ioctl(fd,request,arg) << endl;
  } while(r == -1);
  return r;
}

IplImage* getImage() {
  int fd = open("/dev/video0", O_NONBLOCK);
  if(fd == -1)
  {
    perror("Opening Video device didn't work");
    return NULL;
  }
  struct v4l2_capability caps = {0};
  if(-1 == xioctl(fd,VIDIOC_QUERYCAP, &caps))
  {
    perror("Couldn't query camera");
    return NULL;
  }
  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = 640;
  fmt.fmt.pix.height = 480;
  if(-1 == xioctl(fd,VIDIOC_S_FMT, &fmt))
  {
    perror("Couldn't set resolution");
    return NULL;
  }
  
  struct v4l2_requestbuffers req = {0};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if(-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
  {
    perror("Couldn't request buffer");
    return NULL;
  }
  struct v4l2_buffer buf = {0};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
  {
    perror("Couldn't query buffer");
    return NULL;
  }
  uint8_t* buffer = (uint8_t*) mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
  
  if(!camActive) {
    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
      perror("Start capture error");
      return NULL;
    }

    FD_ZERO(&fds);
    FD_SET(fd,&fds);
    camActive = true;
  }
  struct timeval tv = {0};
  tv.tv_sec = 2;
  int r = select(fd+1, &fds, NULL, NULL, &tv);
  if(-1 == r)
  {
    perror("Waiting for frame");
    return NULL;
  }
  cout << "Got to this other place" << endl;
  if(-1 == xioctl(fd,VIDIOC_DQBUF, &buf))
  {
    cout << errno << endl;
    perror("Retrieving Frame error");
    return NULL;
  }
  cout << "Got the frame" << endl;
  CvMat cvmat = cvMat(fmt.fmt.pix.width, fmt.fmt.pix.height, CV_8UC3, (void*) buffer);
  IplImage *img;
  cout << "made the image" << endl;
  img = cvDecodeImage(&cvmat, 1);
  return img;
}

Mat getBWImage(Mat intrinsic, Mat distCoeffs) {
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
  cout << "about to get image" << endl;
  img = getImage();
  imshow("img", img);
  cout << "Got image" << endl;
  //Undistortion processing
  undistort(img, imgFixed, intrinsic, distCoeffs);
  //imshow( "image", imgFixed );
  cout << "Finished undistort" << endl;
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
  cout << "Using OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;  
  
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
    cout << "Here1" << endl;
    dilatedImg = getBWImage(getIntrinsic(), getDistCoeffs());
    cout << "Here2" << endl;
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
        goodContours.push_back(contours[i]);
        goodRect.push_back(boundRect[i]);
      }
    }
/*    while(goodRect.size() > 1) {
      goodRect.pop_back();
      goodContours.pop_back();
    }*/

    //Draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3 );
    for(unsigned int i = 0; i < goodContours.size(); i++)
    {
      rectangle(drawing, goodRect[i].tl(), goodRect[i].br(), color, 2,8,0);
      drawContours(drawing, goodContours, i, color, 2,8,hierarchy, 0, Point() );
    }
     imshow("Contours", drawing);
    //Determine which contour to use.



    //We'll assume there's only one contour for now, but this needs to be fixed.
      if(goodRect.size() > 0) {
        const double alpha = 0.5*(asin(2*z*goodRect[0].height / (f*h_naught)));
        const double d = z/sin(alpha);
        const double beta = acos(d * goodRect[0].width / (f * w_naught));
        const double beta_h = asin(sin(beta) / cos(alpha));
         
        cout << "New image: " << endl;
        cout << "\t Alpha: \t" << alpha << endl;
        cout << "\t d: \t" << d << endl;
        cout << "\t beta: \t" << beta << endl;
        cout << "\t beta_h: " << beta_h << endl;
        cout << "\t height: " << goodRect[0].height << endl;
        cout << "\t width: \t" << goodRect[0].width << endl;
	
      }
    cout << 1/((clock() - t)*1.0/CLOCKS_PER_SEC) << "fps" << endl;
    waitKey(1);
  }
  return 0;
}
