#include <ros/ros.h>
#include "control_handle.hpp"

typedef ns_control::ControlHandle ControlHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "control");
  ros::NodeHandle nodeHandle("~");
  ControlHandle myControlHandle(nodeHandle);
  ros::Rate loop_rate(myControlHandle.getNodeRate());
  while (ros::ok()) {

    myControlHandle.run();

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

