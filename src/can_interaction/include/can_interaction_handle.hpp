#ifndef CAN_INTERACTION_HANDLE_HPP
#define CAN_INTERACTION_HANDLE_HPP

#include<iostream>
#include "fsd_common_msgs/ControlCommand.h"
#include "std_msgs/Int8.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


namespace ns_can_interaction {

class Can_interactionHandle {

 public:
  // Constructor
  Can_interactionHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
  void can_init();
  void can_write();
  void can_read();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber controlCmdSubscriber_;
  ros::Subscriber errorSignalSubscriber_;
  ros::Publisher resSignalPublisher_;
  void controlCmdCallback(const fsd_common_msgs::ControlCommand &msg);
  void errorSignalCallback(const std_msgs::Int8 &msg);
  std::string res_sign_topic_name_;
  std::string error_signal_topic_name_;
  std::string controlCmd_topic_name_;

  int node_rate_;
  int s, nbytes, nbytes_error_, nbytes_read;
  int can_id_write_;
  int can_dlc_write_;
  int can_id_read_;
  int transmission_ratio_;
  int test_angle_;
  fsd_common_msgs::ControlCommand cmd_;
  int motor_torque_;
  std_msgs::Int8 res_link_info_;
  std_msgs::Int8 error_signal_;
  bool is_init_ = false;
  int max_torque_;
};
}

#endif //CAN_INTERACTION_HANDLE_HPP
