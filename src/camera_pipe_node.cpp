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

int main(int argc, char** argv){

  ros::init(argc, argv, "camera_pipe_node");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image/all", 1);
  
	IMGPipe p("/tmp/_ap0102_pixel", 4*1280, 960);
	cv::namedWindow("RGB");
	
	/*
	int counter=0;
	for (int i = 0; ; ++i) {
		auto rgb = p.get_rgb();
		cv::imshow("RGB", rgb);
		char c=cv::waitKey(5);
    }

*/
  
  ros::Rate loop_rate(30);
  
  while (nh.ok()) {

    auto rgb = p.get_rgb();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb).toImageMsg();
    pub.publish(msg);

    cv::imshow("RGB", rgb);
		cv::waitKey(1);
		
    loop_rate.sleep();
  }
  
  return 0;
}

