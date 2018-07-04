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

using namespace cv;
using namespace std;

std::vector<string> cam_list = {"front", "rear", "left", "right"}; // cam 0,1,2,3

int main(int argc, char** argv){

  ros::init(argc, argv, "camera_pipe_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  if(argc < 5){
    ROS_ERROR("Four args are needed: path_to_IMGPipe, number of cameras, image_width, image_height");
    return 1;
  }

  std::string path_IMGPipe = argv[1]; //Default: "/tmp/_ap0102_pixel"
  int cam_num = atoi(argv[2]); //Default: 4
  int image_width = atoi(argv[3]); //Default: 1280
  int image_height = atoi(argv[4]); //Default: 960
  
  image_transport::Publisher pub_all = it.advertise("camera/image/all", 1); //all four images

  // initialize publishers for each camera
  std::vector<image_transport::Publisher> pub_cam_vec;
  pub_cam_vec.resize(cam_num);

  for(int i = 0; i < cam_num; i++){
    std::string topic = "camera/image/" + cam_list[i];
    pub_cam_vec[i] = it.advertise(topic.c_str(), 1);
  }

  // establish the pipe
  IMGPipe p(path_IMGPipe.c_str(), cam_num*image_width, image_height);
	
  ros::Rate loop_rate(50);
  
  while(nh.ok()) {

    cv::Mat all = p.get_rgb();
    if(all.cols != cam_num*image_width){
      ROS_ERROR("Camera numbers does not match the setting.");
      return 2;
    }

    sensor_msgs::ImagePtr msg_all = cv_bridge::CvImage(std_msgs::Header(), "bgr8", all).toImageMsg();
    pub_all.publish(msg_all);

    std::vector<cv::Mat> img_vec;
    img_vec.resize(cam_num);

    for(int i = 0; i < cam_num; i++){
      img_vec[i] = all(cv::Rect(i*image_width, 0, image_width, image_height));
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_vec[i]).toImageMsg();
      image_transport::Publisher pub = pub_cam_vec[i];
      pub.publish(msg);

      std::string cam_name = cam_list[i];
//      cv::imshow(cam_name, img_vec[i]);
    }

//    cv::imshow("All", all);
		cv::waitKey(1);
		
    loop_rate.sleep();
  }
  
  return 0;
}

