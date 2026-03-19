#ifndef TF_BROADCASTER_HANDLE_HPP
#define TF_BROADCASTER_HANDLE_HPP

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/String.h"

namespace ns_tf_broadcaster {

class Tf_broadcasterHandle {

 public:
  // Constructor
  Tf_broadcasterHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  // void publishToTopics();
  void run();
  void tf_broadcaster_set();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber tfInfoSubscriber_;
  ros::Subscriber localizationWaySubscriber_;
  ros::Subscriber missionSubscriber_;

  void tfInfoCallback(const sensor_msgs::PointCloud &msg);
  void localizationWayCallback(const std_msgs::String &msg);
  void missionCallback(const std_msgs::String &msg);
  
  std::string tf_info_sub_topic_name_;
  std::string localization_way_sub_topic_name_;
  std::string mission_topic_name_;
  std::string localization_;
  std::string mission_;

  int node_rate_;
  sensor_msgs::PointCloud tf_info_;
  tf::TransformBroadcaster broadcaster_;
  tf::Transform laser_link2base_link;
  tf::Transform slam_map2laser_link;
  tf::Transform slam_map2map;
  tf::Transform imu_link2base_link;
  tf::Transform imu_map2imu_link;
  tf::Quaternion q_laser_link2base_link;
  tf::Quaternion q_slam_map2laser_link;
  tf::Quaternion q_slam_map2map;
  tf::Quaternion q_imu_link2base_link;
  tf::Quaternion q_imu_map2imu_link;

};
}

#endif //TF_BROADCASTER_HANDLE_HPP
