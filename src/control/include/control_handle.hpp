#ifndef CONTROL_HANDLE_HPP
#define CONTROL_HANDLE_HPP

#include "control.hpp"

namespace ns_control {

class ControlHandle {

 public:
  // Constructor
  ControlHandle(ros::NodeHandle &nodeHandle);

  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber carStateSubscriber_;
  ros::Subscriber endPointSubscriber_;
  ros::Subscriber trajSubscriber_;
  ros::Publisher  controlCommandPublisher_;
  ros::Publisher  trajectoryPathPublisher_;
  ros::Publisher  targetPathPublisher_;
  ros::Publisher  predictPathPublisher_;

  void carStateCallback(const fsd_common_msgs::CarState &msg);
  void planningTrajCallback(const geometry_msgs::PoseArray &msg);

  std::string car_state_topic_name_;
  std::string acceleration_track_topic_name_;
  std::string skidpad_track_topic_name_;
  std::string control_command_topic_name_;
  std::string trajectory_path_topic_name_;
  std::string target_path_topic_name_;
  std::string predict_path_topic_name_;
  std::string traj_planning_topic_name_;

  int node_rate_;

  Control control_;
};
}
#endif
