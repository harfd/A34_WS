#include <ros/ros.h>
#include "tf_broadcaster_handle.hpp"

namespace ns_tf_broadcaster {

// Constructor
Tf_broadcasterHandle::Tf_broadcasterHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
}

// Getters
int Tf_broadcasterHandle::getNodeRate() const { return node_rate_; }

// Methods
void Tf_broadcasterHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("tf_info_sub_topic_name",
                                      tf_info_sub_topic_name_,
                                      "/tf/info")) {
    ROS_WARN_STREAM("Did not load tf_info_sub_topic_name. Standard value is: " << tf_info_sub_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("localization_way_sub_topic_name",
                                      localization_way_sub_topic_name_,
                                      "/localization/way")) {
    ROS_WARN_STREAM("Did not load localization_way_sub_topic_name. Standard value is: " << localization_way_sub_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("mission_topic_name",
                                      mission_topic_name_,
                                      "/mission")) {
    ROS_WARN_STREAM("Did not load mission_topic_name. Standard value is: " << mission_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void Tf_broadcasterHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  tfInfoSubscriber_    = nodeHandle_.subscribe(tf_info_sub_topic_name_, 1, &Tf_broadcasterHandle::tfInfoCallback, this);
  localizationWaySubscriber_ = nodeHandle_.subscribe(localization_way_sub_topic_name_, 1, &Tf_broadcasterHandle::localizationWayCallback, this);
  missionSubscriber_ = nodeHandle_.subscribe(mission_topic_name_, 1, &Tf_broadcasterHandle::missionCallback, this);
}

void Tf_broadcasterHandle::tfInfoCallback(const sensor_msgs::PointCloud &msg) {
  tf_info_ =  msg;
  if (tf_info_.points.size() != 0) {
    // ROS_INFO("laser_link2base_link (X : %.2f,Y : %.2f, yaw : %.2f)", tf_info_.points[0].x,tf_info_.points[0].y,tf_info_.points[0].z);
    // ROS_INFO("slam_map2laser_link  (X : %.2f,Y : %.2f, yaw : %.6f)", tf_info_.points[1].x,tf_info_.points[1].y,tf_info_.points[1].z);
    // ROS_INFO("slam_map2map         (X : %.2f,Y : %.2f, yaw : %.6f)", tf_info_.points[2].x,tf_info_.points[2].y,tf_info_.points[2].z);
  }
  else {
    ROS_WARN("The tf_info is empty!");
  }
}

void Tf_broadcasterHandle::localizationWayCallback(const std_msgs::String &msg) {
  localization_ = msg.data;
}

void Tf_broadcasterHandle::missionCallback(const std_msgs::String &msg) {
  mission_ = msg.data;
}

void Tf_broadcasterHandle::run() {
  tf_broadcaster_set();
}

void Tf_broadcasterHandle::tf_broadcaster_set() {
    if (mission_ == "acceleration" || mission_ == "skidpad") {
        if (tf_info_.points.size() !=0 ) {
            if (localization_ == "slam") {
              q_laser_link2base_link.setRPY(0,0,tf_info_.points[0].z);
              laser_link2base_link.setRotation(q_laser_link2base_link);
              laser_link2base_link.setOrigin(tf::Vector3(tf_info_.points[0].x,tf_info_.points[0].y,0));

              q_slam_map2laser_link.setRPY(0,0,tf_info_.points[1].z);
              slam_map2laser_link.setRotation(q_slam_map2laser_link);
              slam_map2laser_link.setOrigin(tf::Vector3(tf_info_.points[1].x,tf_info_.points[1].y,0));

              q_slam_map2map.setRPY(0,0,tf_info_.points[2].z);
              slam_map2map.setRotation(q_slam_map2map);
              slam_map2map.setOrigin(tf::Vector3(tf_info_.points[2].x,tf_info_.points[2].y,0));

              broadcaster_.sendTransform(tf::StampedTransform(laser_link2base_link,ros::Time::now(),"laser_link","base_link_vehicle"));
              broadcaster_.sendTransform(tf::StampedTransform(slam_map2laser_link,ros::Time::now(),"slam_map","laser_link"));
              broadcaster_.sendTransform(tf::StampedTransform(slam_map2map,ros::Time::now(),"slam_map","fssim_map"));
            }
            else if (localization_ == "imu") {
              q_laser_link2base_link.setRPY(0,0,tf_info_.points[0].z);
              laser_link2base_link.setRotation(q_laser_link2base_link);
              laser_link2base_link.setOrigin(tf::Vector3(tf_info_.points[0].x,tf_info_.points[0].y,0));

              q_slam_map2map.setRPY(0,0,tf_info_.points[2].z);
              slam_map2map.setRotation(q_slam_map2map);
              slam_map2map.setOrigin(tf::Vector3(tf_info_.points[2].x,tf_info_.points[2].y,0));

              q_imu_map2imu_link.setRPY(0,0,-tf_info_.points[3].z);
              imu_map2imu_link.setRotation(q_imu_map2imu_link);
              imu_map2imu_link.setOrigin(tf::Vector3(-tf_info_.points[3].x,-tf_info_.points[3].y,0));

              q_imu_link2base_link.setRPY(0,0,0);
              imu_link2base_link.setRotation(q_imu_link2base_link);
              imu_link2base_link.setOrigin(tf::Vector3(0,0,0));

              broadcaster_.sendTransform(tf::StampedTransform(slam_map2map,ros::Time::now(),"laser_map","fssim_map"));
              broadcaster_.sendTransform(tf::StampedTransform(laser_link2base_link,ros::Time::now(),"laser_map","imu_map"));//laser_map == slam_map
              broadcaster_.sendTransform(tf::StampedTransform(imu_map2imu_link,ros::Time::now(),"imu_map","imu_link"));
              broadcaster_.sendTransform(tf::StampedTransform(imu_link2base_link,ros::Time::now(),"imu_link","base_link_vehicle"));
            }
        }
    }
    else if (mission_ == "trackdrive") {
          q_laser_link2base_link.setRPY(0,0,tf_info_.points[0].z);
          laser_link2base_link.setRotation(q_laser_link2base_link);
          laser_link2base_link.setOrigin(tf::Vector3(tf_info_.points[0].x,tf_info_.points[0].y,0));
          broadcaster_.sendTransform(tf::StampedTransform(laser_link2base_link,ros::Time::now(),"laser_link","base_link_vehicle"));
        }
    }

}