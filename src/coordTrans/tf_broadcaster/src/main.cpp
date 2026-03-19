#include <ros/ros.h>
#include "tf_broadcaster_handle.hpp"

typedef ns_tf_broadcaster::Tf_broadcasterHandle Tf_broadcasterHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nodeHandle("~");
  Tf_broadcasterHandle myTf_broadcasterHandle(nodeHandle);
  ros::Rate loop_rate(myTf_broadcasterHandle.getNodeRate());
  while (ros::ok()) {
    myTf_broadcasterHandle.run();
    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

