
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

  pub_birdview_seg_ = it_.advertise("camera/image/birdview_seg", 1); //birdview images for segmentation

  pub_birdview_park_ = it_.advertise("camera/image/birdview_park", 1); //birdview images for parking lot detection

  img_vec_.resize(cam_num_);

  p_ = new IMGPipe(pipe.c_str(), cam_num_*width_, height_);

}

CameraPipe::~CameraPipe(){
  delete p_;
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

//set the path of birdview map for segmentation
void CameraPipe::setMapPathForSeg(std::string path_XMap, std::string path_YMap){
  path_XMap_seg_ = path_XMap;
  path_YMap_seg_ = path_YMap;

  initBirdviewMapForSeg();
}

//set the path of birdview map for parking lot detection
void CameraPipe::setMapPathForPark(std::string path_XMap, std::string path_YMap){
  path_XMap_park_ = path_XMap;
  path_YMap_park_ = path_YMap;

  initBirdviewMapForPark();
}

// initialize the birdview map for segmentation
void CameraPipe::initBirdviewMapForSeg(){

  cv::FileStorage fs;
  fs = cv::FileStorage(path_XMap_seg_, cv::FileStorage::READ);
  fs["XMap"] >> XMap_seg_;
  fs.release();
  fs = cv::FileStorage(path_YMap_seg_, cv::FileStorage::READ);
  fs["YMap"] >> YMap_seg_;
  fs.release();
  cv::convertMaps(XMap_seg_, YMap_seg_, XMap_seg_, YMap_seg_, CV_16SC2);
}

// initialize the birdview map for parking
void CameraPipe::initBirdviewMapForPark(){

  cv::FileStorage fs;
  fs = cv::FileStorage(path_XMap_park_, cv::FileStorage::READ);
  fs["XMap"] >> XMap_park_;
  fs.release();
  fs = cv::FileStorage(path_YMap_park_, cv::FileStorage::READ);
  fs["YMap"] >> YMap_park_;
  fs.release();
  cv::convertMaps(XMap_park_, YMap_park_, XMap_park_, YMap_park_, CV_16SC2);
}

// generate the birdview image for segmentation and parking
bool CameraPipe::generateBirdviewImage(){

  cv::Mat concatResult;
  cv::vconcat(img_vec_, cam_num_, concatResult);
  cv::remap(concatResult, birdviewImg_seg_, XMap_seg_, YMap_seg_, CV_INTER_LINEAR);
  cv::remap(concatResult, birdviewImg_park_, XMap_seg_, YMap_seg_, CV_INTER_LINEAR);

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

    generateBirdviewImage();

    sensor_msgs::ImagePtr msg_birdview_seg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", birdviewImg_seg_).toImageMsg();
    pub_birdview_seg_.publish(msg_birdview_seg);

    sensor_msgs::ImagePtr msg_birdview_park = cv_bridge::CvImage(std_msgs::Header(), "bgr8", birdviewImg_park_).toImageMsg();
    pub_birdview_park_.publish(msg_birdview_park);

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

  cv::imshow("birdviewImg_seg", birdviewImg_seg_);
  cv::imshow("birdviewImg_park_", birdviewImg_park_);

  cv::waitKey(1);

}
