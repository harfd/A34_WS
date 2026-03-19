#pragma once
#include "ros/ros.h"

struct Weight {
  double x;
  double y;
  double yaw;
  double lateral_error;
  double yaw_error;
  double v;
  double steering_angle;
  double throttle;
  double steering_rate;
  double throttle_rate;
};

struct Param {
  Weight weight;
  int N;
  double dt;
  bool simulation;
  double car_length;
  double initial_velocity;
  double interval;
  double forward_distance;
  double circle_radius;
  double look_ahead;
  double max_lat_acc;
  double desire_vel;

  void getParams(ros::NodeHandle &nh,const std::string &controller) {
    car_length = nh.param("car_length", 1.88);
    N = nh.param("N", 40);
    dt = nh.param("dt", 0.04);
    weight.x = nh.param("weight/x", 3);
    weight.y = nh.param("weight/y", 10);
    weight.yaw = nh.param("weight/yaw", 8);
    weight.lateral_error = nh.param("weight/lateral_error", 1);
    weight.yaw_error = nh.param("weight/yaw_error", 4);
    weight.v = nh.param("weight/v", 0.4);
    weight.steering_angle = nh.param("weight/steering_angle", 10);
    weight.throttle = nh.param("weight/throttle", 10);
    weight.steering_rate = nh.param("weight/steering_rate", 2000);
    weight.throttle_rate = nh.param("weight/throttle_rate", 10);
    simulation = nh.param("simulation", true);
    interval = nh.param("interval", 0.08);
    forward_distance = nh.param("forward_distance", 15.0);
    circle_radius = nh.param("circle_radius", 9.125);
    look_ahead = nh.param("look_ahead", 10);
    max_lat_acc = nh.param("max_lat_acc", 3.0);
    if (controller == "mpc")
      desire_vel = nh.param("weight/desire_vel", 15);
    else
      desire_vel = nh.param("desire_vel", 3);
  }
};

extern Param param_;
