#ifndef GetImage_H
#define GetImage_H
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

class GetImage
{
	
    public:
        GetImage(void);
        void open_device();
        void init_device();
        void start_capturing();
        cv::Mat mainloop();
        void stop_capturing();
        void uninit_device();
        void close_device();
};

#endif
