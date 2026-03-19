/*
version:    4.5
author:     Shawn Pan 
Time:       2021.10.24 22:05

debug: 对lidar_bbox_msgs容器erase，会造成索引不对齐的问题，新建了lidar_bbox_msgs和raw_lidar_bbox_msgs_区分
增加投影框可视化图像话题/fusion_img,可视化cone_idx,img_idx
rviz可视化marker：配对的cone_idx以及img_idx，未配对的显示-1，cone_idx
增加对投影回图像的cone_bbox阈值
更改融合点云显示为pointcloud<XYZRGB>

*/


#include <ros/ros.h>
#include "fusion_handle.hpp"

namespace ns_fusion {

// Constructor
FusionHandle::FusionHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    fusion_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  getCalibration_Param();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int FusionHandle::getNodeRate() const { return node_rate_; }

// Methods
void FusionHandle::loadParameters() {
    ROS_INFO("loading handle parameters");
    //image_topic
    if (!nodeHandle_.param<std::string>("image_topic_name",
                                        image_topic_name_,
                                        "/darknet_ros/detection_image")) {
        ROS_WARN_STREAM("Did not load image_topic_name. Standard value is: " << image_topic_name_);
    }
    //lidar_topic
    if (!nodeHandle_.param<std::string>("lidar_topic_name",
                                        lidar_topic_name_,
                                        "/perception/cluster_cones")) {
        ROS_WARN_STREAM("Did not load lidar_bbox_topic_name. Standard value is: " << lidar_topic_name_);
    }
    //lidar_bbox_topic
    if (!nodeHandle_.param<std::string>("lidar_bbox_topic_name",
                                        lidar_bbox_topic_name_,
                                        "/perception/clustermsgs")) {
        ROS_WARN_STREAM("Did not load lidar_bbox_topic_name. Standard value is: " << lidar_bbox_topic_name_);
    }
    //output_topic
    if (!nodeHandle_.param<std::string>("color_cloud_topic_name_",
                                        color_cloud_topic_name_,
                                        "/perception/color_cloud")) {
        ROS_WARN_STREAM("Did not load color_cloud_topic_name. Standard value is: " << color_cloud_topic_name_);
    }
    if (!nodeHandle_.param<std::string>("color_marker_topic_name_",
                                        color_marker_topic_name_,
                                        "/perception/color_marker")) {
        ROS_WARN_STREAM("Did not load color_marker_topic_name. Standard value is: " << color_marker_topic_name_);
    }
    if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
        ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
    }
    //calibration param
    if (!nodeHandle_.param<std::string>("calibration_file",
                                        calibration_file_,
                                        "20191011_left_lidar_camera_calibration.yaml")) {
        ROS_WARN_STREAM("Did not load calibration_file_. Standard value is: " << calibration_file_);
    }
    //bbox_topic
    if (!nodeHandle_.param<std::string>("bbox_topic_name",
                                        bbox_topic_name_,
                                        "/darknet_ros/bounding_boxes")) {
        ROS_WARN_STREAM("Did not load img_bbox_topic. Standard value is: " << bbox_topic_name_);
    }
}

void FusionHandle::subscribeToTopics() {
  
  //debug
  //rawimageSubscriber_ = nodeHandle_.subscribe(image_topic_name_, 1, &FusionHandle::rawimageCallback, this);

  imageSubscriber_.subscribe(nodeHandle_, image_topic_name_, 1);
  lidar_bbox_sub_.subscribe(nodeHandle_,lidar_bbox_topic_name_,1);
  bboxSubscriber_.subscribe(nodeHandle_,bbox_topic_name_,1);
  sync_.connectInput(imageSubscriber_, lidar_bbox_sub_, bboxSubscriber_);
  sync_.registerCallback(boost::bind(&FusionHandle::TimesyncCallback,this, _1, _2,_3));

  lidarSubscriber_ = nodeHandle_.subscribe(lidar_topic_name_, 1, &FusionHandle::lidarCallback, this);
  //rawLidarSubscriber_ = nodeHandle_.subscribe(raw_lidar_topic_name_, 1, &LidarClusterHandle::rawLidarCallback, this);
  
  pathSubscriber_ = nodeHandle_.subscribe("/visualization/visual_traj" , 1 , &FusionHandle::pathCallback, this);
  ROS_INFO("subscribe to topics");
}

void FusionHandle::publishToTopics() {
  color_cloud_Pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(color_cloud_topic_name_, 1);
  color_marker_Pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(color_marker_topic_name_, 1);
  map_pub_ = nodeHandle_.advertise<fsd_common_msgs::Map>("/cone_map", 1);
  image_transport::ImageTransport it(nodeHandle_);
  fusion_img_pub_ = it.advertise("/fusion_img",1);
  ROS_INFO("publish to topics");
}

void FusionHandle::run() {
  std::chrono::steady_clock::time_point   t2 = std::chrono::steady_clock::now();
	fusion_.runAlgorithm();
  std::chrono::steady_clock::time_point   t1 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
  std::cout<<"porcessing_time: "<<time_round<<std::endl;
  if (time_round > 0.5){
    ROS_INFO("large porcessing_time!");
  }
  sendMsg();
}

void FusionHandle::sendMsg() {
  color_cloud_Pub_.publish(fusion_.getColorCloud());
  color_marker_Pub_.publish(fusion_.getColorMarker());
  map_pub_.publish(fusion_.getFusionMap());
  fusion_img_pub_.publish(fusion_.getFusionImage());
}
void FusionHandle::getCalibration_Param(){
  struct stat buffer;
  if ( stat(calibration_file_.c_str(),&buffer) != 0 ){
    ROS_ERROR("calibration files do not exits!");
  }
  fusion_.getcalibration(calibration_file_);
  ROS_DEBUG_STREAM("get calibration file");
}

void FusionHandle::TimesyncCallback(const sensor_msgs::ImageConstPtr &image_msg ,
                                    const fusion::ClusterConesConstPtr  &lidar_msg,
                                    const fusion::BoundingBoxes::ConstPtr &bbox_msg) {
  ROS_DEBUG_STREAM("READY TO Set Fusion_msg");
  fusion_.setFusion_msg(image_msg,lidar_msg,bbox_msg);
}

void FusionHandle::rawimageCallback(const sensor_msgs::ImageConstPtr &rawimage_msg) {
  ROS_DEBUG_STREAM("READY TO Set rawimage_msg");
}

void FusionHandle::lidarCallback(const sensor_msgs::PointCloud2 &cluster_cone_msg) {
  fusion_.setCluster_Cone_msg(cluster_cone_msg);
  ROS_DEBUG_STREAM("READY TO Set cluster_cone_msg");
}

void FusionHandle::pathCallback(const visualization_msgs::MarkerArray &path_msg){
  fusion_.setPath_msg(path_msg);
  ROS_DEBUG_STREAM("READY TO Set path_msg");
}


}