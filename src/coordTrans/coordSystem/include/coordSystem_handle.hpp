#ifndef COORDSYSTEM_HANDLE_HPP
#define COORDSYSTEM_HANDLE_HPP

#include "coordSystem.hpp"

namespace ns_coordSystem {

class CoordSystemHandle {

 public:
  // Constructor
  CoordSystemHandle(ros::NodeHandle &nodeHandle);

  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void sendMsg();
  void run();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber centerPointsSubscriber_;
  ros::Subscriber imuStateSubscriber_;
  ros::Subscriber slamStateSubscriber_;
  ros::Publisher  tfInfoPublihser_;
  ros::Publisher  imuErrorSignalPublisher_;
  ros::Publisher  imuCoordPublisher_;
  ros::Publisher  localizationWayPublisher_;
  ros::Publisher  missionPublisher_;
  ros::Publisher  visualTrianglesPublisher_;
  ros::Publisher  visualPathPublisher_;

  void imuStateCallback(const coordSystem::imu_state &msg);
  void centerPointsCallback(const sensor_msgs::PointCloud &msg);
  void slamStateCallback(const geometry_msgs::Pose2D &msg);

  std::string imu_state_topic_name_;
  std::string center_points_topic_name_;
  std::string slam_state_topic_name_;
  std::string tf_info_topic_name_;
  std::string imu_error_signal_topic_name_;
  std::string imu_coord_topic_name_;
  std::string localization_way_topic_name_;
  std::string mission_topic_name_;
  std::string visual_triangles_topic_name_;
  std::string visual_path_topic_name_;

  int node_rate_;

  CoordSystem coordSystem_;

};
}

#endif //COORDSYSTEM_HANDLE_HPP
