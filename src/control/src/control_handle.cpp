#include <ros/ros.h>
#include "control_handle.hpp"

namespace ns_control {

// Constructor
ControlHandle::ControlHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    control_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int ControlHandle::getNodeRate() const { return node_rate_; }

// Methods
void ControlHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("car_state_topic_name",
                                      car_state_topic_name_,
                                      "/estimation/slam/state")) {
    ROS_WARN_STREAM("Did not load car_state_topic_name. Standard value is: " << car_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("control_command_topic_name",
                                      control_command_topic_name_,
                                      "/control/pure_pursuit/control_command")) {
    ROS_WARN_STREAM("Did not load control_command_topic_name. Standard value is: " << control_command_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("trajectory_path_topic_name",
                                      trajectory_path_topic_name_,
                                      "/path/trajectory")) {
    ROS_WARN_STREAM("Did not load trajectory_path_topic_name. Standard value is: " << trajectory_path_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("target_path_topic_name",
                                      target_path_topic_name_,
                                      "/path/target")) {
    ROS_WARN_STREAM("Did not load target_path_topic_name. Standard value is: " << target_path_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("predict_path_topic_name_",
                                      predict_path_topic_name_,
                                      "/path/predict")) {
    ROS_WARN_STREAM("Did not load predict_path_topic_name. Standard value is: " << predict_path_topic_name_);
  }
    if (!nodeHandle_.param<std::string>("traj_planning_topic_name_",
                                      traj_planning_topic_name_,
                                      "/planing/traj")) {
    ROS_WARN_STREAM("Did not load traj_planning_topic_name_. Standard value is: " << traj_planning_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 100)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void ControlHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  carStateSubscriber_ = nodeHandle_.subscribe(car_state_topic_name_, 1, &ControlHandle::carStateCallback, this);
  trajSubscriber_     = nodeHandle_.subscribe(traj_planning_topic_name_, 1, &ControlHandle::planningTrajCallback, this);
}

void ControlHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  controlCommandPublisher_ = nodeHandle_.advertise<fsd_common_msgs::ControlCommand>(control_command_topic_name_, 1);
  trajectoryPathPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(trajectory_path_topic_name_, 1);
  targetPathPublisher_     = nodeHandle_.advertise<visualization_msgs::MarkerArray>(target_path_topic_name_, 1);
  predictPathPublisher_    = nodeHandle_.advertise<visualization_msgs::MarkerArray>(predict_path_topic_name_, 1);
}

void ControlHandle::run() {
	control_.runAlgorithm();
  sendMsg();
}

void ControlHandle::sendMsg() {
  controlCommandPublisher_.publish(control_.getControlCommand());
  trajectoryPathPublisher_.publish(control_.getTrajectoryPath());
  targetPathPublisher_.publish(control_.getTargetPath());
  predictPathPublisher_.publish(control_.getPredictPath());
}


void ControlHandle::carStateCallback(const fsd_common_msgs::CarState &msg) {
  control_.setCarState(msg);
}

void ControlHandle::planningTrajCallback(const geometry_msgs::PoseArray &msg) {
  control_.setPlanningTraj(msg);
}

}