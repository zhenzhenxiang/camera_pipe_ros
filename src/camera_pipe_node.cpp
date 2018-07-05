
//opencv
#include <opencv2/opencv.hpp>

//ros
#include <ros/ros.h>

//user
#include "camera_pipe.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv){

  ros::init(argc, argv, "camera_pipe_node");
  ros::NodeHandle nh;

  std::string path_IMGPipe = nh.param("camera_pipe_path", std::string("/tmp/_ap0102_pixel"));
  int cam_num = nh.param("cam_num", 4);
  int image_width = nh.param("image_width", 1280);
  int image_height = nh.param("image_height", 960);

  CameraPipe cp(nh, path_IMGPipe, cam_num, image_width, image_height);

  cp.run();
  
  return 0;
}

