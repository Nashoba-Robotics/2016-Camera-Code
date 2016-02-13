#ifndef GetImage_H
#define GetImage_H


class GetImage
{
	
    public:
        GetImage(void);
        void open_device();
        void init_device();
        void start_capturing();
        IplImage* mainloop();
        void stop_capturing();
        void uninit_device();
        void close_device();
};

#endif
