
#include "camera_pipe.h"

CameraPipe::CameraPipe(NodeHandle &nh, std::string pipe, int cam_num, int width, int height):
  nh_(nh), it_(nh), cam_num_(cam_num), width_(width), height_(height), info_manager_(nh){

  pub_all_ = it_.advertise("camera/image/all", 1); //all four images

  cam_list_ = {"left", "right", "front", "rear"}; // cam 0,1,2,3

  pub_cam_vec_.resize(cam_num_);
  for(int i = 0; i < cam_num_; i++){
    std::string topic = "camera/image/" + cam_list_[i];
    pub_cam_vec_[i] = it_.advertise(topic.c_str(), 1);
  }

  pub_birdview_seg_ = it_.advertise("camera/image/birdview_seg", 1); //birdview images for segmentation

  pub_birdview_park_ = it_.advertise("camera/image/birdview_park", 1); //birdview images for parking lot detection

  pub_front_undistored_ = it_.advertise("camera/image/front_ssd", 1); //undistorted front cam image for ssd

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
bool CameraPipe::setMapPathForSeg(std::string path_XMap, std::string path_YMap){
  path_XMap_seg_ = path_XMap;
  path_YMap_seg_ = path_YMap;

  return initBirdviewMapForSeg();
  
}

//set the path of birdview map for parking lot detection
bool CameraPipe::setMapPathForPark(std::string path_XYMap){
  path_XYMap_park_ = path_XYMap;

  return initBirdviewMapForPark();
}

//initialize the birdview map for segmentation
bool CameraPipe::initBirdviewMapForSeg(){

  cv::Mat tmp_XMap, tmp_YMap;

  cv::FileStorage fs;
  fs = cv::FileStorage(path_XMap_seg_, cv::FileStorage::READ);
  fs["XMap"] >> tmp_XMap;
  fs.release();
  fs = cv::FileStorage(path_YMap_seg_, cv::FileStorage::READ);
  fs["YMap"] >> tmp_YMap;
  fs.release();
  
  if(tmp_XMap.empty() || tmp_YMap.empty()){
    ROS_ERROR("Failed to load birdview map for segmentation!");
    return false;
  }
  
  cv::convertMaps(tmp_XMap, tmp_YMap, XMap_seg_, YMap_seg_, CV_16SC2);
  return true;

}

//initialize the birdview map for parking
bool CameraPipe::initBirdviewMapForPark(){

  cv::Mat tmp_XMap, tmp_YMap;
  
  cv::FileStorage fs(path_XYMap_park_, cv::FileStorage::READ);
  fs["XMap"] >> tmp_XMap;
  fs["YMap"] >> tmp_YMap;
  fs.release();
  
  if(tmp_XMap.empty() || tmp_YMap.empty()){
    ROS_ERROR("Failed to load birdview map for parking!");
    return false;
  }
  
  cv::convertMaps(tmp_XMap, tmp_YMap, XMap_park_, YMap_park_, CV_16SC2);
  return true;
}

//generate the birdview image for segmentation and parking
bool CameraPipe::generateBirdviewImage(){
  
  cv::Mat concatResult;
  cv::vconcat(img_vec_, concatResult);
  cv::remap(concatResult, birdviewImg_seg_, XMap_seg_, YMap_seg_, CV_INTER_LINEAR);
  cv::remap(concatResult, birdviewImg_park_, XMap_park_, YMap_park_, CV_INTER_LINEAR);

  return !birdviewImg_seg_.empty() && !birdviewImg_park_.empty();

}

//load camera info from yaml file
bool CameraPipe::loadCameraInfo(std::string path_cam_info, std::string cam_name){

  bool success = false;
  info_manager_.setCameraName(cam_name);

  if (info_manager_.validateURL(path_cam_info)){

    if(info_manager_.loadCameraInfo(path_cam_info)){
      info_ = info_manager_.getCameraInfo();

      //camera intrinsic matrix
      const cv::Mat K(3,3,CV_64F,&info_.K.front());
      /*
      const cv::Mat camera_kMatrix = (cv::Mat_<double>(3,3) <<  314.7547755462885, 0, 644.5649061478241,
                                                                 0, 326.9831749345991, 519.4605613151753,
                                                                 0, 0, 1);
      */

      //distort matrix
      const cv::Mat D(1,4,CV_64F,&info_.D.front());
      //const cv::Mat camera_distortMatrix = (cv::Mat_<double>(1,4) << 0.0803433, 0.0494984, -0.0386655, 0.00642292);

      cv::Size image_size(width_, height_);
      XMap_front_ = Mat(image_size, CV_32FC1);
      YMap_front_ = Mat(image_size, CV_32FC1);
      cv::Mat R = Mat::eye(3,3,CV_32F);
      cv::fisheye::initUndistortRectifyMap(K, D, R,
                                           getOptimalNewCameraMatrix(K, D,
                                                                     image_size, 1, image_size, 0),
                                           image_size, CV_32FC1, XMap_front_, YMap_front_);
      success = true;
    }
    else{
      ROS_ERROR("Failed to load camera info, file does not exist: %s", path_cam_info.c_str());
      success = false;
    }

  }
  else{
    ROS_ERROR("Failed to resolve the path: %s", path_cam_info.c_str());
    success = false;
  }

  return success;

}

//undistort the image of front camera
bool CameraPipe::undistortFrontImage(){

  //get front camera index and undistort the image
  auto front_cam_it = std::find(cam_list_.begin(), cam_list_.end(), "front");
  if(front_cam_it != cam_list_.end()){
    size_t cam_index = front_cam_it - cam_list_.begin();

    cv::remap(img_vec_[cam_index], undistortedFrontImg_, XMap_front_, YMap_front_, INTER_LINEAR);
    if(!undistortedFrontImg_.empty())
      return true;
    else{
      ROS_ERROR("Failed to undistor front camera image. Exit!");
      return false;
    }
  }
  else{
    ROS_ERROR("Front camera is not defined. Failed to undistort front camera image. Exit.");
    return false;
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

    if(!generateBirdviewImage()){
      ROS_ERROR("Failed to generate birdview image! Exit.");
      return;
    }

    sensor_msgs::ImagePtr msg_birdview_seg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", birdviewImg_seg_).toImageMsg();
    pub_birdview_seg_.publish(msg_birdview_seg);

    sensor_msgs::ImagePtr msg_birdview_park = cv_bridge::CvImage(std_msgs::Header(), "bgr8", birdviewImg_park_).toImageMsg();
    pub_birdview_park_.publish(msg_birdview_park);

    if(!undistortFrontImage()){
      return;
    }

    sensor_msgs::ImagePtr msg_front_undistorted = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistortedFrontImg_).toImageMsg();
    pub_front_undistored_.publish(msg_front_undistorted);

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
  cv::imshow("birdviewImg_park", birdviewImg_park_);

  cv::imshow("undistortedFrontImg", undistortedFrontImg_);

  cv::waitKey(1);

}
