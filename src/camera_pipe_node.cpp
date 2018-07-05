
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
  ros::NodeHandle pnh("~");

  std::string path_IMGPipe = nh.param("camera_pipe_path", std::string("/tmp/_ap0102_pixel"));
  int cam_num = pnh.param("cam_num", 4);
  int image_width = pnh.param("image_width", 1280);
  int image_height = pnh.param("image_height", 960);

  std::string path_birdview_XMap_seg = pnh.param("birdview_XMap_path_seg", std::string("XMap_seg.yaml"));
  std::string path_birdview_YMap_seg = pnh.param("birdview_YMap_path_seg", std::string("YMap_seg.yaml"));

  std::string path_birdview_XYMap_park = pnh.param("birdview_XYMap_path_park", std::string("XYMap_park.yaml"));

  CameraPipe cp(nh, path_IMGPipe, cam_num, image_width, image_height);

  if(cp.setMapPathForSeg(path_birdview_XMap_seg, path_birdview_YMap_seg) &&
    cp.setMapPathForPark(path_birdview_XYMap_park)){

      ROS_INFO("Start processing and republishing camera images...");
      cp.run();

  	}

  return 0;
}

