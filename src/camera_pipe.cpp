
#include "camera_pipe.h"

CameraPipe::CameraPipe(NodeHandle &nh, std::string pipe, int cam_num, int width, int height):
  nh_(nh), it_(nh), cam_num_(cam_num), width_(width), height_(height){

  pub_all_ = it_.advertise("camera/image/all", 1); //all four images

  cam_list_ = {"front", "rear", "left", "right"}; // cam 0,1,2,3

  pub_cam_vec_.resize(cam_num_);
  for(int i = 0; i < cam_num_; i++){
    std::string topic = "camera/image/" + cam_list_[i];
    pub_cam_vec_[i] = it_.advertise(topic.c_str(), 1);
  }

  img_vec_.resize(cam_num_);

  p_ = new IMGPipe(pipe.c_str(), cam_num_*width_, height_);

}

//get image from pipe
bool CameraPipe::getImageFromPipe(){

  img_all_ = p_->get_rgb();

  return img_all_.cols == cam_num_*width_;
}

//seperate images from the all-in-one image for each camera
void CameraPipe::seperateImages(){

  for(int i = 0; i < cam_num_; i++){
    img_vec_[i] = img_all_(cv::Rect(i*width_, 0, width_, height_));
  }

}

//run the publishing process
void CameraPipe::run(){

  while(nh_.ok()){

    if(!getImageFromPipe()){
      ROS_ERROR("Can not get camera image from pipe, or cam_num is not correct! Exit.");
      return;
    }

    seperateImages();

    sensor_msgs::ImagePtr msg_all = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_all_).toImageMsg();
    pub_all_.publish(msg_all);

    for(int i = 0; i < cam_num_; i++){
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_vec_[i]).toImageMsg();
      image_transport::Publisher pub = pub_cam_vec_[i];
      pub.publish(msg);
    }

    visualize();

    ros::Duration(0.03).sleep();
  }

}

//visualize the images
void CameraPipe::visualize(){

  cv::imshow("All", img_all_);

  for(int i = 0; i < cam_num_; i++){
    std::string cam_name = cam_list_[i];
    cv::imshow(cam_name, img_vec_[i]);
  }

  cv::waitKey(1);

}
