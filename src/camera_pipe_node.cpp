
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

  if(argc < 5){
    ROS_ERROR("Four args are needed: path_to_IMGPipe, number of cameras, image_width, image_height");
    return 1;
  }

  std::string path_IMGPipe = argv[1]; //Default: "/tmp/_ap0102_pixel"
  int cam_num = atoi(argv[2]); //Default: 4
  int image_width = atoi(argv[3]); //Default: 1280
  int image_height = atoi(argv[4]); //Default: 960

  CameraPipe cp(nh, path_IMGPipe, cam_num, image_width, image_height);

  cp.run();
  
  return 0;
}

