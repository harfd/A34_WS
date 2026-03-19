#include <ros/ros.h>
#include "can_interaction_handle.hpp"

namespace ns_can_interaction {

// Constructor
Can_interactionHandle::Can_interactionHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int Can_interactionHandle::getNodeRate() const { return node_rate_; }

// Methods
void Can_interactionHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("controlCmd_topic_name",
                                      controlCmd_topic_name_,
                                      "/control/pure_pursuit/control_command")) {
    ROS_WARN_STREAM("Did not load controlCmd_topic_name. Standard value is: " << controlCmd_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("res_sign_topic_name",
                                      res_sign_topic_name_,
                                      "/res/linked_signal")) {
    ROS_WARN_STREAM("Did not load res_sign_topic_name. Standard value is: " << res_sign_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("error_signal_topic_name",
                                      error_signal_topic_name_,
                                      "/error_signal")) {
    ROS_WARN_STREAM("Did not load error_signal_topic_name. Standard value is: " << error_signal_topic_name_);
  }
  if (!nodeHandle_.param("motor_torque", motor_torque_, 0)) {
    ROS_WARN_STREAM("Did not load motor_torque. Standard value is: " << motor_torque_);
  }
  if (!nodeHandle_.param("can_id_write", can_id_write_, 0x11)) {
    ROS_WARN_STREAM("Did not load can_id_write. Standard value is: " << can_id_write_);
  }
  if (!nodeHandle_.param("can_dlc_write", can_dlc_write_, 4)) {
    ROS_WARN_STREAM("Did not load can_dlc_write. Standard value is: " << can_dlc_write_);
  }
  if (!nodeHandle_.param("can_id_read", can_id_read_, 0x1E4)) {
    ROS_WARN_STREAM("Did not load can_id_read_. Standard value is: " << can_id_read_);
  }
  if (!nodeHandle_.param("transmission_ratio", transmission_ratio_, 5)) {
    ROS_WARN_STREAM("Did not load transmission_ratio. Standard value is: " << transmission_ratio_);
  }
  if (!nodeHandle_.param("test_angle", test_angle_, 0)) {
    ROS_WARN_STREAM("Did not load test_angle. Standard value is: " << test_angle_);
  }
  if (!nodeHandle_.param("max_torque", max_torque_, 20)) {
    ROS_WARN_STREAM("Did not load max_torque. Standard value is: " << max_torque_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void Can_interactionHandle::can_init() {
  struct sockaddr_can addr;
  struct ifreq ifr;
  strcpy(ifr.ifr_name, "can0" );
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  ioctl(s, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));
}

void Can_interactionHandle::can_write() {
  struct can_frame frame;
  frame.can_id = can_id_write_;
  frame.can_dlc = can_dlc_write_;
  float steering_angle = (cmd_.steering_angle.data)*180/3.1415926; //rad to deg
  float throttle = cmd_.throttle.data;

  //judge the steering angle direction
  if (steering_angle >= 0) 
    frame.data[0] = 0x00;
  else
    frame.data[0] = 0xFF;
  //calculate the steering wheel angle size 
  frame.data[1] = int(std::fabs(steering_angle)*transmission_ratio_);
  // frame.data[1] = test_angle_;

  //judge the whether do the acceleration or deceleration
  if (throttle >= 0)
  frame.data[2] = 0x00;
  else
  frame.data[2] = 0xFF;
  //control the drive motor
  if (res_link_info_.data == 77 && motor_torque_ <= max_torque_) 
    frame.data[3] = motor_torque_;
  else
    frame.data[3] = 0;

  if (error_signal_.data == 2 || error_signal_.data == 1) {
    ROS_WARN("error !\n");
    struct can_frame frame_error;
    frame_error.can_id = can_id_read_;
    frame_error.can_dlc = 1;
    frame_error.data[0] = 0x13;
    nbytes_error_ = write(s, &frame_error, sizeof(frame_error));
  }
  //write the imformation to the can bus
  nbytes = write(s, &frame, sizeof(frame));

  printf("\nsteering angle : %.2f ; throttle : %.2f ; motor torque : %d . \n",steering_angle, throttle, frame.data[3]);
  if(nbytes != sizeof(frame))
    ROS_WARN("The can device doesn't work! \n");
}

void Can_interactionHandle::can_read() {
  struct can_frame frame;
  struct can_filter frame_filter;
  frame_filter.can_id = can_id_read_;
  frame_filter.can_mask = CAN_SFF_MASK;
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &frame_filter, sizeof(frame_filter));
  nbytes_read = read(s, &frame, sizeof(frame));
  if (nbytes_read > 0) {
    printf("res signal ID: 0x%X ,DLC: %d, data: 0x%X \n", frame.can_id, frame.can_dlc, frame.data[0]);
    if (frame.data[0] == 0x11 && is_init_ == false) {
      res_link_info_.data = 77;
      is_init_ = true;
      ROS_INFO("The res linked successfully!");
    }
  }
  else
    ROS_WARN("Didn't read the imformation for the can bus!");

}

void Can_interactionHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  controlCmdSubscriber_ = nodeHandle_.subscribe(controlCmd_topic_name_, 1, &Can_interactionHandle::controlCmdCallback, this);
  errorSignalSubscriber_ = nodeHandle_.subscribe(error_signal_topic_name_, 1, &Can_interactionHandle::errorSignalCallback, this);
}

void Can_interactionHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  resSignalPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(res_sign_topic_name_,1);
}

void Can_interactionHandle::run() {
  ros::Rate loop_rate(node_rate_);
  while (ros::ok()) {
    can_read();
    sendMsg();
    can_write();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Can_interactionHandle::sendMsg() {
  resSignalPublisher_.publish(res_link_info_);
}

void Can_interactionHandle::controlCmdCallback(const fsd_common_msgs::ControlCommand &msg) {
  cmd_ = msg;
}

void Can_interactionHandle::errorSignalCallback(const std_msgs::Int8 &msg) {
  error_signal_ = msg;
}
}
