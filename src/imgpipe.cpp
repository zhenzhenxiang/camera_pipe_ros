#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <cerrno>
#include <thread>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "imgpipe.hpp"

class IMGPipe_impl
{
    char *planar;
    char *packed;
    char *conv;

    int got_length;

    const int dim_422p = 2;
    const int dim_yuy2 = 2;
    const int dim_rgb = 3;

    int width;
    int height;

    int pipe;
    std::thread fetch;
    std::mutex mt;

private:
    void _422p_to_yuy2(const char *_422p, char *yuy2, const int w, const int h) {
        const char *Y = _422p;
        const char *U = Y + w * h;
        const char *V = U + w * h / 2;
        // do not lock mutex, it will cause low performance, although teared image may happen
        for (int i = 0; i < w * h; ++i) {
            yuy2[i * 2] = Y[i];
            yuy2[i * 2 + 1] = (i & 1) == 0 ? U[i / 2] : V[i / 2];
        }
    }

    void fetch_pipe_async() {
        while (1) {
            mt.lock();
            if (pipe < 0) {
                return;
            }
            got_length = read(pipe, planar, width * height * dim_422p);
            mt.unlock();
            if (got_length == 0) {
                usleep(10000); // do not use cpu when pipe is empty
            }
        }
    }

public:
    IMGPipe_impl(const char *fifo, const int w, const int h) {
        got_length = 0;

        if (access(fifo, F_OK) == -1) {
            if (mkfifo(fifo, 0644) < 0) {
                printf("mkfifo: %s\n", strerror(errno));
            }
        }

        printf("Waiting for pipe writer...");
        fflush(stdout);

        pipe = open(fifo, O_RDONLY);
        if (pipe < 0) {
            printf("open: %s\n", strerror(errno));
        }

        printf("established\n");
        fflush(stdout);

        width = w;
        height = h;

        int sz = w * h;

        planar = new char[sz * dim_422p];
        packed = new char[sz * dim_yuy2];
        conv = new char[sz * dim_rgb];

        fetch = std::thread(&IMGPipe_impl::fetch_pipe_async, this);
    }

    ~IMGPipe_impl() {
        mt.lock();
        delete[] planar;
        delete[] packed;
        delete[] conv;

        close(pipe);
        pipe = -1;
        mt.unlock();
        fetch.join();
    }

    cv::Mat get_yuy2() {
        if (got_length) {
            _422p_to_yuy2(planar, packed, width, height);
        }
        cv::Mat ret(cv::Size(width, height), CV_8UC2, packed);
        return ret.clone();
    }

    cv::Mat get_rgb() {
        cv::Mat ret(cv::Size(width, height), CV_8UC3, conv);
        if (got_length) {
            cv::Mat yuv(get_yuy2());
            cv::cvtColor(yuv, ret, CV_YUV2RGB_YUY2);
        }
        return ret.clone();
    }

    cv::Mat get_gray() {
        cv::Mat ret(cv::Size(width, height), CV_8UC1, planar);
        return ret.clone();
    }
};


// bridge class as interface

IMGPipe::IMGPipe(const char *fifo, const int w, const int h)
    : _impl(new IMGPipe_impl(fifo, w, h)) {}
IMGPipe::~IMGPipe()
{
    delete _impl;
}
cv::Mat IMGPipe::get_yuy2()
{
    return _impl->get_yuy2();
}
cv::Mat IMGPipe::get_rgb()
{
    return _impl->get_rgb();
}
cv::Mat IMGPipe::get_gray()
{
    return _impl->get_gray();
}

