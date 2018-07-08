#ifndef CAMERA_PIPE_H
#define CAMERA_PIPE_H

//system
#include <ctime>
#include <algorithm>
//opencv
#include <opencv2/opencv.hpp>

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

//user
#include "imgpipe.hpp"

using namespace std;
using namespace ros;
using namespace cv;

class CameraPipe{
public:
  CameraPipe(ros::NodeHandle &nh, std::string pipe, int cam_num, int width, int height);
  ~CameraPipe();

  bool setMapPathForSeg(std::string path_XMap, std::string path_YMap);
  bool setMapPathForPark(std::string path_XYMap);
  bool loadCameraInfo(std::string path_cam_info, std::string cam_name);

  void run();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Publisher pub_all_;
  std::vector<image_transport::Publisher> pub_cam_vec_;

  image_transport::Publisher pub_birdview_seg_;
  image_transport::Publisher pub_birdview_park_;

  image_transport::Publisher pub_front_undistored_;

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

  std::string path_XYMap_park_;

  cv::Mat XMap_park_;
  cv::Mat YMap_park_;

  cv::Mat birdviewImg_seg_;
  cv::Mat birdviewImg_park_;

  cv::Mat XMap_front_;
  cv::Mat YMap_front_;
  cv::Mat undistortedFrontImg_;

  sensor_msgs::CameraInfo info_;
  camera_info_manager::CameraInfoManager info_manager_;

  bool getImageFromPipe();
  void seperateImages();
  bool initBirdviewMapForSeg();
  bool initBirdviewMapForPark();
  bool generateBirdviewImage();
  bool undistortFrontImage();
  void visualize();

};
#endif // CAMERA_PIPE_H
