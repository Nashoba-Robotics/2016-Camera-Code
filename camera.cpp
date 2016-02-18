#include <cv.hpp>
#include <highgui.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/gpu/gpu.hpp>
#include <cmath>
#include "GetImage.h"
#include "network.h"

// Network buffer size
#define BUFFER 1024

//#define UseUnDistort
#define UseThresholding
#define UseDilation

//#define ShowWindows

#ifdef UseThresholding
#define UseContours
#endif

#define SmartCapture

#define Rotate

#define Log

//#define r640x480
//#define r1280x720
#define r1920x1080

#ifdef r1280x720
#ifdef Rotate
#define WIDTH 720
#define HEIGHT 1280
#else
#define WIDTH 1280
#define HEIGHT 720
#endif
#elif defined(r640x480)
#ifdef Rotate
#define WIDTH 480
#define HEIGHT 640
#else
#define WIDTH 640
#define HEIGHT 480
#endif
#elif defined(r1920x1080)
#ifdef Rotate
#define WIDTH 1080
#define HEIGHT 1920
#else
#define WIDTH 1920
#define HEIGHT 1080
#endif
#endif

#ifdef Rotate
#define FOVH 60
#else
#define FOVH 92
#endif

#define HNAUGHT = 12;
#define WNAUGHT = 20;

using namespace cv;
using namespace std;
using namespace gpu;

#ifndef SmartCapture
VideoCapture capture;
#else
GetImage getimg;
#endif

#ifdef UseUnDistort
Mat intrinsic;
Mat distCoeffs;
#endif

static const cv::Scalar ColorWhite   = cv::Scalar( 255, 255, 255 );
static const cv::Scalar ColorPurple  = cv::Scalar( 255, 0, 255 );
static const cv::Scalar ColorGray    = cv::Scalar( 64, 64, 64 );
static const cv::Scalar ColorBlue    = cv::Scalar( 255, 0, 0 );
static const cv::Scalar ColorGreen   = cv::Scalar( 0, 255, 0 );
static const cv::Scalar ColorRed     = cv::Scalar( 0, 0, 255 );
static const cv::Scalar ColorYellow  = cv::Scalar( 0, 255, 255 );

#ifdef UseUnDistort
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
#endif

Mat getImage() {
  Mat img;
#ifdef SmartCapture
  img = getimg.mainloop();
#else
  capture >> img;
#endif

#ifdef SmartCapture
  img = getimg.mainloop();
#else
  capture >> img;
#endif
 
#ifdef Rotate
  transpose(img,img);
  flip(img,img,1);
#endif

  return img;
}



Mat getBWImage(Mat img) {
#ifdef UseDilation
  //Dilation
  const int dilationSize = 2;
  const Mat dilateElement = getStructuringElement(MORPH_RECT, Size(2*dilationSize + 1, 2*dilationSize + 1), Point(dilationSize, dilationSize));
#endif

  //HLS Thresholding
  const int H_low = 60;
  const int H_high = 90;
  const int S_high = 255;
  const int S_low = 78;
  const int L_low = 50;
  const int L_high = 200;
  const Scalar low = Scalar(H_low, L_low, S_low);
  const Scalar high = Scalar(H_high, L_high, S_high);

  Mat imgFixed;
  Mat imgThresh;
  Mat dilatedImg;
  GpuMat hls;

#ifdef UseUnDistort
  //Undistortion processing
  undistort(img, imgFixed, intrinsic, distCoeffs);
  GpuMat imgFixedGpu(imgFixed);
#else
  GpuMat imgFixedGpu(img);
#endif
#ifdef UseThresholding
  //HLS Threshold processing
  gpu::cvtColor(imgFixedGpu, hls, COLOR_BGR2HLS);
  inRange(Mat(hls), low, high, imgThresh);
#else
  imgThresh = Mat(imgFixedGpu);
#endif
#ifdef UseDilation
  dilate(imgThresh, dilatedImg, dilateElement);
#else
  dilatedImg = imgThresh;
#endif
#ifdef ShowWindows
  imshow("dilate", dilatedImg);
#endif
  return dilatedImg;
}

int main(int argc, char* argv[])
{
#ifdef Log
  cout << "Using OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;
#endif
  int numGPU = getCudaEnabledDeviceCount();
#ifdef Log
  cout << "Number of GPU Devices: " << numGPU << endl;
#endif
  if(numGPU == 0) {
#ifdef Log
    cout << "Need to have a GPU Device" << endl;
#endif
    return 1;
  }
  
  setDevice(0);

#ifdef UseUnDistort
  setDistCoeffs();
  setIntrinsic();
#endif

#ifdef SmartCapture
  getimg = GetImage();
  getimg.open_device();
  getimg.init_device();
  getimg.start_capturing();
#else
  capture = VideoCapture(0);
#endif
  //Contours
  const int minArea = 500;
  const int thresh = 200; //For edge detection 
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  const int poly_epsilon = 10;
  
  while(1)
  {
#ifdef Log
    clock_t t = clock();
#endif
    Mat img = getImage();
    GpuMat dilatedImg = GpuMat(getBWImage(img));
    //Contours processing
#ifdef UseThresholding
#ifdef UseContours
    GpuMat canny_output;
    gpu::Canny(dilatedImg, canny_output, thresh, thresh*2, 3 );
    findContours( Mat(canny_output), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    /// Find the convex hull object for each contour
    vector<vector<Point> > hull( contours.size() );
    for(unsigned int i = 0; i < contours.size(); i++ ) {
      convexHull( Mat(contours[i]), hull[i], false );
    }
    // Approximate the convex hulls with polygons
    // This reduces the number of edges and makes the contours
    // into quads
    vector<vector<Point> > poly( contours.size() );
    for (unsigned int i=0; i < contours.size(); i++) {
      approxPolyDP(hull[i], poly[i], poly_epsilon, true);
      // These come out reversed, so reverse back
      reverse(poly[i].begin(), poly[i].end());
    }
    // Prune the polygons into only the ones that we are intestered in.
    vector<vector<Point> > prunedPoly(0);
    vector<vector<Point> > prunedHulls(0);
    vector<vector<Point> > prunedContours(0);
    if (poly.size() > 0) {
      int size = minArea;
      int largestPos = WIDTH;
      int largest = -1;
      for (unsigned int i=0; i < poly.size(); i++) {
        Rect bRect = boundingRect(poly[i]);
        // Remove polygons that are too small
        if (bRect.width * bRect.height > minArea) {
          prunedPoly.push_back(poly[i]);
          prunedHulls.push_back(hull[i]);
          prunedContours.push_back(contours[i]);
          if(bRect.width * bRect.height > size-200 && bRect.width/2 + bRect.x < largestPos ) {
            size = bRect.width * bRect.height;
            largest = prunedPoly.size() - 1;
            largestPos = bRect.width/2 + bRect.x;
          }
        }
      }
      //There are no targest bigger than the minArea
      if(largest == -1)
        goto FPS;
      vector<Point> goodPoly = prunedPoly[largest];
#ifdef ShowWindows
      // Output the final image
      Mat finalImage = img.clone();
      for (unsigned int i=0; i < prunedPoly.size(); i++) {
        drawContours(finalImage, prunedPoly, i, ColorBlue, 1, 8, vector<Vec4i>(), 0, Point() );
      }
      
      imshow("Contours", finalImage);
#endif
      //Determine the topLeft corner x
      const int tlcornerX = min(min(goodPoly[0].x,goodPoly[1].x),min(goodPoly[2].x,goodPoly[3].x));
      const int tlcornerY = min(min(goodPoly[0].y,goodPoly[1].y),min(goodPoly[2].y,goodPoly[3].y));

      //Determine the width and height   
      const double x1 = (abs(goodPoly[0].x - goodPoly[1].x) + abs(goodPoly[2].x - goodPoly[3].x))/2;
      const double x2 = (abs(goodPoly[1].x - goodPoly[2].x) + abs(goodPoly[3].x - goodPoly[0].x))/2;
      const double width = max(x1,x2);

      const double y1 = (abs(goodPoly[0].y - goodPoly[1].y) + abs(goodPoly[2].y - goodPoly[3].y))/2;
      const double y2 = (abs(goodPoly[1].y - goodPoly[2].y) + abs(goodPoly[3].y - goodPoly[0].y))/2;
      const double height = max(y1,y2);

      double leftHeight;
      double rightHeight;
      if(height == y1) {
        if(goodPoly[0].x < goodPoly[2].x) { //0 and 1 are the left two corners
          leftHeight = abs(goodPoly[0].y - goodPoly[1].y);
          rightHeight = abs(goodPoly[2].y - goodPoly[3].y);
        } else { //2 and 3 are the left two corners
          leftHeight = abs(goodPoly[2].y - goodPoly[3].y);
          rightHeight = abs(goodPoly[0].y - goodPoly[1].y);
        }
      } else {
        if(goodPoly[0].x < goodPoly[2].x) { //1 and 2 are the left two corners
          leftHeight = abs(goodPoly[1].y - goodPoly[2].y);
          rightHeight = abs(goodPoly[0].y - goodPoly[3].y);
        } else { //3 and 0 are the left two corners
          leftHeight = abs(goodPoly[0].y - goodPoly[3].y);
          rightHeight = abs(goodPoly[2].y - goodPoly[1].y);
        }
      }

      if(leftHeight > 200 || rightHeight > 200) {
        goto FPS;
      }

      const double xCenterOfTarget = width/2.0 + tlcornerX;
      const double yCenterOfTarget = height/2.0 + tlcornerY;
      const double leftRightPixels = xCenterOfTarget - WIDTH/2.0;
      const double turn = (FOVH/(1.0 * WIDTH)) * leftRightPixels ;
        
      const double distance = (height-128)/(-3.7);
      if(distance < 0 || distance > 25 || abs(turn) > 40) {
        goto FPS;
      }
#ifdef Log   
      cout << "New image: " << endl;
      cout << "\tleftHeight: " << leftHeight << endl;
      cout << "\trightHeight: " << rightHeight << endl;
      cout << "\txCenter of target: " << xCenterOfTarget << endl;
      cout << "\tyCenter of target: " << yCenterOfTarget << endl;
      cout << "\tleftRightPixels: " << leftRightPixels << endl;
      cout << "\tdistance: \t" << distance << endl;
      cout << "\tpos turn means we need to turn counterclockwise" << endl;
      cout << "\tturn: \t" << turn << endl;
      cout << "\theight: " << height << endl;
      cout << "\twidth: \t" << width << endl;
      cout << "\tx: \t" << tlcornerX << endl;
      cout << "\ty: \t" << tlcornerY << endl;
#endif
      sendMessageRect("10.17.68.33", distance, turn); 
    }
#endif
#endif
    FPS:
#ifdef Log
    char fps[1024];
    sprintf(fps, "%f fps", CLOCKS_PER_SEC*1.0/(clock() - t));
    cout << fps << endl;
    sendMessage("10.17.68.33", fps);
#endif
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
