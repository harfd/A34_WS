#ifndef ESTIMATIONSTATE_HANDLE_HPP
#define ESTIMATIONSTATE_HANDLE_HPP

#include "std_msgs/String.h"
#include "coordSystem/imu_state.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/Cone.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

namespace ns_estimationState {

class EstimationStateHandle {

 public:
  // Constructor
  EstimationStateHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
  fsd_common_msgs::CarState setCarState();
  fsd_common_msgs::Map setLocalMap();
  fsd_common_msgs::CarState setTrackdriveCarState();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber imuStateSubscriber_;
  ros::Subscriber tfInfoSubscriber_;
  ros::Subscriber imuCoordSubscriber_;
  ros::Subscriber localizationWaySubscriber_;
  ros::Subscriber missionSubscriber_;
  ros::Subscriber coneMapSubscriber_;
  ros::Publisher carStatePublisher_;
  ros::Publisher localMapPublisher_;

  void imuStateCallback(const coordSystem::imu_state &msg);
  void tfInfoCallback(const sensor_msgs::PointCloud &msg);
  void imuCoordCallback(const geometry_msgs::Point &msg);
  void localizationWayCallback(const std_msgs::String &msg);
  void coneMapCallback(const fsd_common_msgs::Map &msg);
  void missionCallback(const std_msgs::String &msg);

  std::string imu_state_topic_name_;
  std::string tf_info_sub_topic_name_;
  std::string imu_coord_sub_topic_name_;
  std::string localization_way_sub_topic_name_;
  std::string cone_map_sub_topic_name_;
  std::string car_state_topic_name_;
  std::string local_map_topic_name_;
  std::string mission_topic_name_;

  coordSystem::imu_state imu_state_;
  sensor_msgs::PointCloud tf_info_;
  fsd_common_msgs::CarState car_state_;
  tf::TransformListener listener_;
  tf::TransformListener listener1_;
  tf::StampedTransform transform_;
  tf::StampedTransform transform1_;

  int node_rate_;
  int coordSystemNum_;
  bool is_init_data_;
  std::ofstream coordFileSlam_;
  std::ofstream coordFileImu_;
  double time_init_;
  geometry_msgs::Point imu_coord_;
  std::string localization_;
  std::string mission_;
  fsd_common_msgs::Map cone_map_;
  fsd_common_msgs::Map local_map_;
  std::string slamCoordFile_;
  std::string imuCoordFile_;

};
}

#endif //ESTIMATIONSTATE_HANDLE_HPP
