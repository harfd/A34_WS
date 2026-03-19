#include <ros/ros.h>
#include "coordSystem_handle.hpp"

namespace ns_coordSystem {

// Constructor
CoordSystemHandle::CoordSystemHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    coordSystem_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int CoordSystemHandle::getNodeRate() const { return node_rate_; }

// Methods
void CoordSystemHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("imu_state_topic_name",
                                      imu_state_topic_name_,
                                      "/sensor/imu")) {
    ROS_WARN_STREAM("Did not load imu_state_topic_name. Standard value is: " << imu_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("center_points_topic_name",
                                      center_points_topic_name_,
                                      "/perception/center_point")) {
    ROS_WARN_STREAM("Did not load center_points_topic_name_. Standard value is: " << center_points_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("slam_state_topic_name",
                                      slam_state_topic_name_,
                                      "/slam/state")) {
    ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << slam_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("tf_info_topic_name",
                                      tf_info_topic_name_,
                                      "/tf/info")) {
    ROS_WARN_STREAM("Did not load tf_info_topic_name. Standard value is: " << tf_info_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("imu_coord_topic_name",
                                      imu_coord_topic_name_,
                                      "/imu/coord")) {
    ROS_WARN_STREAM("Did not load imu_coord_topic_name. Standard value is: " << imu_coord_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("localization_way_topic_name",
                                      localization_way_topic_name_,
                                      "/localization/way")) {
    ROS_WARN_STREAM("Did not load localization_way_topic_name. Standard value is: " << localization_way_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("mission_topic_name",
                                      mission_topic_name_,
                                      "/mission")) {
    ROS_WARN_STREAM("Did not load mission_topic_name. Standard value is: " << mission_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("imu_error_signal_topic_name",
                                      imu_error_signal_topic_name_,
                                      "/error/signal")) {
    ROS_WARN_STREAM("Did not load imu_error_signal_topic_name. Standard value is: " << imu_error_signal_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("visual_triangles_topic_name",
                                      visual_triangles_topic_name_,
                                      "/visual/triangles")) {
    ROS_WARN_STREAM("Did not load visual_triangles_topic_name. Standard value is: " << visual_triangles_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("visual_path_topic_name",
                                      visual_path_topic_name_,
                                      "/visual/path")) {
    ROS_WARN_STREAM("Did not load visual_path_topic_name. Standard value is: " << visual_path_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void CoordSystemHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  imuStateSubscriber_  = nodeHandle_.subscribe(imu_state_topic_name_,  1, &CoordSystemHandle::imuStateCallback, this);
  centerPointsSubscriber_ = nodeHandle_.subscribe(center_points_topic_name_,  1, &CoordSystemHandle::centerPointsCallback, this);
  slamStateSubscriber_ = nodeHandle_.subscribe(slam_state_topic_name_,  1, &CoordSystemHandle::slamStateCallback, this);
}

void CoordSystemHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  tfInfoPublihser_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(tf_info_topic_name_, 1);
  imuErrorSignalPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(imu_error_signal_topic_name_, 1);
  imuCoordPublisher_ = nodeHandle_.advertise<geometry_msgs::Point>(imu_coord_topic_name_, 1);
  localizationWayPublisher_ = nodeHandle_.advertise<std_msgs::String>(localization_way_topic_name_, 1);
  missionPublisher_ = nodeHandle_.advertise<std_msgs::String>(mission_topic_name_, 1);
  visualTrianglesPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(visual_triangles_topic_name_, 1);
  visualPathPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(visual_path_topic_name_, 1);
}

void CoordSystemHandle::run() {
	coordSystem_.runAlgorithm();
  sendMsg();
}

void CoordSystemHandle::sendMsg() {
  tfInfoPublihser_.publish(coordSystem_.getTfInfo());
  imuErrorSignalPublisher_.publish(coordSystem_.getErrorSignal());
  imuCoordPublisher_.publish(coordSystem_.getImuCoord());
  localizationWayPublisher_.publish(coordSystem_.getLocalizationWay());
  missionPublisher_.publish(coordSystem_.getMission());
  visualTrianglesPublisher_.publish(coordSystem_.getTriangles());
  visualPathPublisher_.publish(coordSystem_.getPath());
}

void CoordSystemHandle::imuStateCallback(const coordSystem::imu_state &msg) {
  coordSystem_.setImuState(msg);
}

void CoordSystemHandle::centerPointsCallback(const sensor_msgs::PointCloud &msg) {
  coordSystem_.setCenterPoints(msg);
}

void CoordSystemHandle::slamStateCallback(const geometry_msgs::Pose2D &msg) {
  coordSystem_.setSlamState(msg);
}


}