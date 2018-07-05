#ifndef CAMERA_PIPE_H
#define CAMERA_PIPE_H

//system
#include <ctime>

//opencv
#include <opencv2/opencv.hpp>

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//user
#include "KeyControl.hpp"
#include "imgpipe.hpp"

using namespace std;
using namespace ros;
using namespace cv;

class CameraPipe{
public:
  CameraPipe(ros::NodeHandle &nh, std::string pipe, int cam_num, int width, int height);
  ~CameraPipe();

  bool setMapPathForSeg(std::string path_XMap, std::string path_YMap);
  bool setMapPathForPark(std::string path_XMap, std::string path_YMap);

  void run();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Publisher pub_all_;
  std::vector<image_transport::Publisher> pub_cam_vec_;

  image_transport::Publisher pub_birdview_seg_;
  image_transport::Publisher pub_birdview_park_;

  IMGPipe *p_;

  cv::Mat img_all_;
  std::vector<cv::Mat> img_vec_;

  int cam_num_;
  int width_;
  int height_;

  std::vector<string> cam_list_;

  std::string path_XMap_seg_;
  std::string path_YMap_seg_;

  cv::Mat XMap_seg_;
  cv::Mat YMap_seg_;

  std::string path_XMap_park_;
  std::string path_YMap_park_;

  cv::Mat XMap_park_;
  cv::Mat YMap_park_;

  cv::Mat birdviewImg_seg_;
  cv::Mat birdviewImg_park_;

  bool getImageFromPipe();
  void seperateImages();
  bool initBirdviewMapForSeg();
  bool initBirdviewMapForPark();
  bool generateBirdviewImage();
  void visualize();

};
#endif // CAMERA_PIPE_H
