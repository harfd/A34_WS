/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "lidar_cluster_handle.hpp"
#include <ClusterCone.h>
#include <ClusterCones.h>

namespace ns_lidar_cluster {

// Constructor
LidarClusterHandle::LidarClusterHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    lidar_cluster_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int LidarClusterHandle::getNodeRate() const { return node_rate_; }

// Methods
void LidarClusterHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("raw_lidar_topic_name",
                                      raw_lidar_topic_name_,
                                      "/velodyne_points_init")) {
    ROS_WARN_STREAM("Did not load raw_lidar_topic_name. Standard value is: " << raw_lidar_topic_name_);
  }

  if (!nodeHandle_.param<std::string>("cones_centroid_topic_name",
                                      cones_centroid_topic_name_,
                                      "/perception/cones_centroid")) {
    ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << cones_centroid_topic_name_);
  }

  if (!nodeHandle_.param<std::string>("filter_cones_topic_name",
                                      filter_cones_topic_name_,
                                      "/perception/filter_cones")) {
    ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << filter_cones_topic_name_);
  }

  if (!nodeHandle_.param<std::string>("filter_ground_topic_name",
                                      filter_ground_topic_name_,
                                      "/perception/filter_ground")) {
    ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << filter_ground_topic_name_);
  }


  if (!nodeHandle_.param<std::string>("cluster_cones_topic_name",
                                      cluster_cones_topic_name_,
                                      "/perception/cluster_cones")) {
    ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << filter_ground_topic_name_);
  }

  if (!nodeHandle_.param("node_rate", node_rate_, 10)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }



}

void LidarClusterHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  rawLidarSubscriber_ = nodeHandle_.subscribe(raw_lidar_topic_name_, 1, &LidarClusterHandle::rawLidarCallback, this);
}

void LidarClusterHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  conesCentroidPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(cones_centroid_topic_name_, 1);
  filterConesPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(filter_cones_topic_name_, 1);
  // filterGroundPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(filter_ground_topic_name_, 1);
  clusterConesPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(cluster_cones_topic_name_, 1);
  clustermsgPublisher_ = nodeHandle_.advertise<lidar_cluster::ClusterCones>("/perception/clustermsgs",1);
  rviz_visual_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/perception/visualization_marker", 1);
  pub_cloud_greedy_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/greedy_points", 10);
  
}

void LidarClusterHandle::run() {
  std::chrono::steady_clock::time_point   t2 = std::chrono::steady_clock::now();
  lidar_cluster_.runAlgorithm();
  std::chrono::steady_clock::time_point   t1 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
  std::cout<<"time: "<<time_round<<std::endl;
  if (time_round > 0.5){
    ROS_INFO("large porcessing_time!");
  }
  sendMsg();
}

void LidarClusterHandle::sendMsg() {
  if(!lidar_cluster_.is_ok())
    return;
  conesCentroidPublisher_.publish(lidar_cluster_.getConesCentroid());
  filterConesPublisher_.publish(lidar_cluster_.getFilterCones());
  // filterGroundPublisher_.publish(lidar_cluster_.getFilterGround());
  clusterConesPublisher_.publish(lidar_cluster_.getClusterCones());
  clustermsgPublisher_.publish(lidar_cluster_.getClusterMsgs());
  rviz_visual_.publish(lidar_cluster_.getRvizVisual());
  // pub_cloud_greedy_.publish(lidar_cluster_.getGreedyCloud());
}

void LidarClusterHandle::rawLidarCallback(const sensor_msgs::PointCloud2 &msg) {
  lidar_cluster_.setRawLidar(msg);
}
}