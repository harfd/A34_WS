#include <ros/ros.h>
#include "control.hpp"
#include <sstream>

namespace ns_control {
// Constructor
Control::Control(ros::NodeHandle &nh) : nh_(nh) {
  mission_ = nh_.param<std::string>("mission", "acceleration");
  controller_ = nh_.param<std::string>("controller", "pure_pursuit");
  acceleration_length_ = nh_.param<double>("acceleration_length",75);
  param_.getParams(nh_,controller_);

  if (mission_ == "acceleration") { track_ = &track_acceleration_; }
  else if (mission_ == "skidpad") { track_ = &track_skidpad_; }
  else if (mission_ == "trackdrive") {track_ = &track_trackdrive_; }

  if (controller_ == "pure_pursuit") { solver_ = &solver_purePursuit_; }
  else if (controller_ == "mpc") { solver_ = &solver_mpc_; }
};
 
// Getters for calculated imformation for publisher
fsd_common_msgs::ControlCommand Control::getControlCommand() { return controlCommand_; }
visualization_msgs::MarkerArray Control::getTrajectoryPath() { return trajectoryPath_; }
visualization_msgs::MarkerArray Control::getTargetPath()     { return targetPath_;     }
visualization_msgs::MarkerArray Control::getPredictPath()    { return predictPath_;    }

// Setters for callback functions of subscriber
void Control::setCarState(const fsd_common_msgs::CarState &msg) { carState_ = msg; }
void Control::setPlanningTraj(const geometry_msgs::PoseArray &msg) {planningTraj_ = msg;}


bool Control::check(){
    if (mission_ == "acceleration") {
        endPoint_.x = acceleration_length_;
        endPoint_.y = 0;
        if (fabs(endPoint_.y) != 0) {
        ROS_WARN_STREAM("Acceleration end point is error !");
        return false;
        }
    if (mission_ == "trackdrive") {
        if (planningTraj_.poses.size() == 0) {
        ROS_WARN_STREAM("The trajectory is empty !");
        }
    }
    }

    else 
    ROS_DEBUG_STREAM("Successfully passing check");
    return true;
}

void Control::setTrack(){
    if (!is_init){
        if (mission_ == "acceleration") {
            track_->setTrackEndPoint(endPoint_);
        }
        track_->getTraj();
    }
    is_init = true;

    if (mission_ == "trackdrive") {
    track_->setPlanningPath(planningTraj_);
    track_->getTraj();
    }
}


void Control::runAlgorithm() {
    if (!check()) { 
        ROS_WARN_STREAM("Check Error"); 
        return;
    }
    setTrack();

    track_->setTrackVehicleState(VehicleState(carState_,controlCommand_));
    track_->getTrajPath(trajectory_);
    track_->calTraj(relativePath_);
    solver_->setSolverState(VehicleState(carState_,controlCommand_));
    solver_->setTrajectory(relativePath_);


    if (mission_ == "acceleration" || mission_ == "skidpad") {
        solver_->solve();
    }
    else {
        if (planningTraj_.poses.size() != 0) {
            solver_->solve();
        }
    }
    if (mission_ == "acceleration" && carState_.car_state.x <= acceleration_length_) {controlCommand_ = solver_->getControlCommand();}
    else if (mission_ == "skidpad" && carState_.car_state.x <= 35) {controlCommand_ = solver_->getControlCommand();}
    else if (mission_ == "trackdrive" && planningTraj_.poses.size() != 0) {controlCommand_ = solver_->getControlCommand();}
    else {controlCommand_.steering_angle.data = 0;
          controlCommand_.throttle.data = -1;}

    std::vector<float> color_traj    = {1, 0, 0};
    std::vector<float> color_targert = {0, 1, 0};
    std::vector<float> color_predict = {0, 0, 1};

    visualize_trajectory(trajectory_, trajectoryPath_, "/fssim_map", color_traj, carState_.header, true, 50);
    visualize_trajectory(relativePath_, targetPath_, "/base_link_vehicle", color_targert, carState_.header, true, 1);
    if (controller_ == "mpc")
    {
    visualize_trajectory(solver_->getTrajectory(), predictPath_, "/base_link_vehicle", color_predict, carState_.header, true, 1);   
    }
    if (planningTraj_.poses.size() != 0) {
        visualize_traj(relativePath_,visualTraj_);
    }
}

}
