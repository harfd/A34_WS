#ifndef IMU_LOAD_HPP
#define IMU_LOAD_HPP

#include <vector>
#include <math.h>
#include <time.h>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "imu_load/imu_state.h"
#include <iostream>
#include <stdlib.h>
#include <bitset>
#include "stdio.h"
#include<sensor_msgs/Imu.h>

namespace ns_imu_load {

class serial_imu_load {

 public:

  serial_imu_load(ros::NodeHandle& nh);

  int serial_init();
  void getAndCheck();
  sensor_msgs::Imu getImuState();
  void publishToTopics();
  void sendMsg();
  short solver_byte2(int startIndex,int byteNum,uint8_t buffer[512]);
  int solver_byte4(int startIndex,int byteNum,uint8_t buffer[512]);


private:

	ros::NodeHandle& nh_;

	imu_load::imu_state imuState;
  
  serial::Serial ser;
  sensor_msgs::Imu imu_state_;
  int baudrate_;
  int node_rate_;
  std::string serial_port_;
  std::string imu_msgs_topic_name_;
  int index_;

  ros::Publisher imuMsgsPublisher_;
  
};
}

#endif //IMU_LOAD_HPP
