
#ifndef __IMGPIPE_HPP__
#define __IMGPIPE_HPP__

namespace cv
{
class Mat;
}

class IMGPipe_impl;

class IMGPipe
{
    IMGPipe_impl *_impl;
public:
    IMGPipe(const char *fifo, const int w, const int h);
    ~IMGPipe();
    cv::Mat get_yuy2();
    cv::Mat get_rgb();
    cv::Mat get_gray();
};

#endif //__IMGPIPE_HPP__

