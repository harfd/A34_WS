#include <ros/ros.h>
#include "can_interaction_handle.hpp"

typedef ns_can_interaction::Can_interactionHandle Can_interactionHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_interaction");
  ros::NodeHandle nodeHandle("~");
  Can_interactionHandle myCan_interactionHandle(nodeHandle);
  ros::Rate loop_rate(myCan_interactionHandle.getNodeRate());

  myCan_interactionHandle.can_init();
  myCan_interactionHandle.run();

  return 0;
}

