/*
version:    4.1
author:     Shawn Pan 
Time:       2021.9.10 16:19
接收话题：image_bbox && lidar_bbox
匈牙利匹配输出赋色marker
*/

#include <ros/ros.h>
#include "fusion_handle.hpp"

typedef ns_fusion::FusionHandle FusionHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion");
  ros::NodeHandle nodeHandle("~");
  FusionHandle myFusionHandle(nodeHandle);
  ros::Rate loop_rate(myFusionHandle.getNodeRate());
  while (ros::ok()) {

    myFusionHandle.run();

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}

