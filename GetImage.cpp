 /*
 * V4L2 video simple capture one frame program Ver.1.00 by saibara
 *
 * Ver.1.00 simple write one yuv file.
 */
#define OUTFILE_NAME	"capture.yuv"
#define COUNT_IGNORE	1	// frame count for initilize camera

#define IMAGE_WIDTH	1920
#define IMAGE_HEIGHT	1080

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#include "GetImage.h"
#define CLEAR(x) memset (&(x), 0, sizeof (x))

using namespace cv;
using namespace std;
  struct buffer {
          void *                  start;
          size_t                  length;
        };

    const char *           dev_name;
    int              fd;
    struct buffer *         buffers;
    unsigned int     n_buffers;
   
    Mat img;
    Mat cmat;
 
    void errno_exit(const char *s)
    {
        fprintf(stderr, "%s error %d, %s\n",s, errno, strerror(errno));  
        exit(EXIT_FAILURE);
    }

    int xioctl(int fd,int request,void *arg)
    {
        int r;
        do{ r = ioctl(fd, request, arg); }
        while(-1 == r && EINTR == errno);
            return r;
    }

    
    GetImage :: GetImage(void )
    {
        dev_name        = NULL; 
        fd              = -1;
        buffers         = NULL;
        n_buffers       = 0;
        dev_name = "/dev/video0";
    }
    
    void process_image(const void *p_buf,const int len_buf)
    {
        
       cmat = Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC3, (void*)p_buf);	
       img = imdecode(cmat, 1);
 //      cout << "refcount: " <<((*img.refcount))<<endl; 
       //cvReleaseMat(img);
	//cout << "img" << &img 
       // delete cvmat;
 //	img = imdecode(p_buf,CV_LOAD_IMAGE_COLOR); 
   }

    int read_frame(int count)   
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
         {
          switch(errno)
            {
              case EAGAIN:
                  return 0;
               case EIO:
                  /* Could ignore EIO, see spec. */
                  /* fall through */
               default:
                  errno_exit("VIDIOC_DQBUF");
                }
              }
          assert(buf.index < n_buffers);
          if(count == 0){
            process_image(buffers[buf.index].start,buffers[buf.index].length);
           // CLEAR(buffers[buf.index].start);
        }
          if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
          
          return 1;
        }
    
    Mat  GetImage ::  mainloop(void)
    {
        unsigned int count;
        count = COUNT_IGNORE;
        
//	img.release();
	while(count-- > 0)
        {
        //fprintf(stderr, "%d count\n",count);
              for(;;)
            {
              fd_set fds;
              struct timeval tv;
              int r;
              FD_ZERO(&fds);
              FD_SET(fd, &fds);
              /* Timeout. */
              tv.tv_sec = 2;
              tv.tv_usec = 0;
              r = select(fd + 1, &fds, NULL, NULL, &tv);
              if(-1 == r)
                {
                  if(EINTR == errno)
	            continue;
                  errno_exit("select");
                }
              if(0 == r)
                {
                  fprintf(stderr, "select timeout\n");
                  //exit(EXIT_FAILURE);
                  return Mat();
                }
              if(read_frame(count))
                break;
              /* EAGAIN - continue select loop. */
            }
        }
        return img;
    }

    void GetImage::stop_capturing (void)
    {
      enum v4l2_buf_type type;
      
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if(-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
    }

    void GetImage::start_capturing(void)
    {
      unsigned int i;
      enum v4l2_buf_type type;
      
      for(i = 0; i < n_buffers; ++i)
          {
	    struct v4l2_buffer buf;
	    CLEAR(buf);
	    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    buf.memory      = V4L2_MEMORY_MMAP;
	    buf.index       = i;
	
	    if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	      errno_exit("VIDIOC_QBUF");
          }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if(-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
    }

    void GetImage::uninit_device(void)
    {
      unsigned int i;
      
      for(i = 0; i < n_buffers; ++i)
        if(-1 == munmap(buffers[i].start, buffers[i].length))
          errno_exit("munmap");
      free(buffers);
    }

    void init_mmap(void)
    {
      struct v4l2_requestbuffers req;
      
      CLEAR(req);
      
      req.count               = 1;
      req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory              = V4L2_MEMORY_MMAP;
      
      if(-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
        {
          if(EINVAL == errno)
	    {
	      fprintf(stderr, "%s does not support "
		      "memory mapping\n", dev_name);
	      exit(EXIT_FAILURE);
	    }
          else
	    {
	      errno_exit("VIDIOC_REQBUFS");
	    }
        }
    /*  
    if(req.count < 2)
        {
          fprintf(stderr, "Insufficient buffer memory on %s\n",
	          dev_name);
          exit(EXIT_FAILURE);
        }
    */
      buffers = (buffer*)calloc(req.count, sizeof(*buffers));
      
      if(!buffers)
        {
          fprintf(stderr, "Out of memory\n");
          exit(EXIT_FAILURE);
        }
      
      for(n_buffers = 0; n_buffers < req.count; ++n_buffers)
        {
          struct v4l2_buffer buf;
          CLEAR(buf);
          buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory      = V4L2_MEMORY_MMAP;
          buf.index       = n_buffers;
          
          if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
	             errno_exit("VIDIOC_QUERYBUF");
      
           printf("Buffer length : %d : %d\n",n_buffers,buf.length);
        
          buffers[n_buffers].length = buf.length;
          buffers[n_buffers].start =
	    mmap(NULL /* start anywhere */,
	         buf.length,
	         PROT_READ | PROT_WRITE /* required */,
	         MAP_SHARED /* recommended */,
	         fd, buf.m.offset);
          
          if(MAP_FAILED == buffers[n_buffers].start)
	    errno_exit("mmap");
        }
    }

    void GetImage::init_device(void)
    {
      struct v4l2_capability cap;
      struct v4l2_cropcap cropcap;
      struct v4l2_crop crop;
      struct v4l2_format fmt;
      unsigned int min;
      
      if(-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
        {
          if(EINVAL == errno)
	    {
	      fprintf(stderr, "%s is no V4L2 device\n",dev_name);
	      exit(EXIT_FAILURE);
	    }
          else
	    {
	      errno_exit("VIDIOC_QUERYCAP");
	    }
        }
      if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
          fprintf(stderr, "%s is no video capture device\n",dev_name);
          exit(EXIT_FAILURE);
        }
      if(!(cap.capabilities & V4L2_CAP_STREAMING))
          {
	    fprintf(stderr, "%s does not support streaming i/o\n",
		    dev_name);
	    exit(EXIT_FAILURE);
          }
      /* Select video input, video standard and tune here. */
      CLEAR(cropcap);
      
      cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      
      if(0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
        {
          crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          crop.c = cropcap.defrect; /* reset to default */
          
          if(-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
	    {
	      switch(errno)
	        {
	        case EINVAL:
	          /* Cropping not supported. */
	          break;
	        default:
	          /* Errors ignored. */
	          break;
	        }
	    }
        }
      else
        {	
          /* Errors ignored. */
        }
      CLEAR(fmt);
      
      fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width       = IMAGE_WIDTH;
      fmt.fmt.pix.height      = IMAGE_HEIGHT;
      //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;

      fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
      
      if(-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) errno_exit("VIDIOC_S_FMT");
      
      /* Note VIDIOC_S_FMT may change width and height. */
      
      /* Buggy driver paranoia. */
      min = fmt.fmt.pix.width * 2;
      if(fmt.fmt.pix.bytesperline < min) fmt.fmt.pix.bytesperline = min;
      min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
      if(fmt.fmt.pix.sizeimage < min) fmt.fmt.pix.sizeimage = min;
      
      init_mmap();
    }

    void GetImage :: close_device(void)
    {
      if(-1 == close(fd)) errno_exit("close");
      fd = -1;
    }

    void GetImage :: open_device(void)
    {
      struct stat st; 
      
      if(-1 == stat(dev_name, &st))
        {
          fprintf(stderr, "Cannot identify '%s': %d, %s\n",
	          dev_name, errno, strerror(errno));
          exit(EXIT_FAILURE);
        }
      
      if(!S_ISCHR(st.st_mode))
        {
          fprintf(stderr, "%s is no device\n", dev_name);
          exit(EXIT_FAILURE);
        }
      fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
      if(-1 == fd)
        {
          fprintf(stderr, "Cannot open '%s': %d, %s\n",
	          dev_name, errno, strerror(errno));
          exit(EXIT_FAILURE);
        }
    }

/*
int main ()
{
   GetImage getimg = GetImage();
 // clock_t t;
  //dev_name = "/dev/video0";
  int i;
  getimg.open_device();
  getimg.init_device();
  getimg.start_capturing();
    for ( i=0; i<50; i++)
    {
   //     t = clock();
      getimg.mainloop();
     //  fprintf(stderr, "fps: %lf\n",1/((clock() - t)*1.0/CLOCKS_PER_SEC));
    } 
      
  getimg.stop_capturing();
  getimg.uninit_device();
  getimg.close_device();
  exit(EXIT_SUCCESS);
  return 0;
}*/

