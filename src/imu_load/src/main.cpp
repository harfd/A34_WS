#include "ros/ros.h"
#include "string.h"
#include "imu_load.hpp"

int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "imu_load");
  ros::NodeHandle nodeHandle;
  ns_imu_load::serial_imu_load serial_imu_load(nodeHandle);
  serial_imu_load.serial_init();
  serial_imu_load.getAndCheck();
  // serial_imu_load.sendMsg();
}
