#include "Track/track_trackdrive.h"
#include "ros/ros.h"

namespace ns_control {
    bool Track_trackdrive::getTraj() {
        if (planningTraj_.poses.size() == 0) {
            ROS_WARN("The planning trajectory is empty!");
            return false;
        }
        fsd::Vec_f wx, wy;
        for (int i = 0; i < planningTraj_.poses.size(); i++) {
            double x = planningTraj_.poses[i].position.x;
            double y = planningTraj_.poses[i].position.y;
            wx.push_back(x);
            wy.push_back(y);
        }
        fsd::Spline2D spline(wx, wy);

        TrajectoryPoint trajectoryPoint;
        trajectory_.clear();
        const double interval = param_.interval;
        for (float i = 0; i < spline.s.back(); i += interval) {
            std::array<float, 2> point_ = spline.calc_postion(i);
            trajectoryPoint.track_x = point_[0];
            trajectoryPoint.track_y = point_[1];
            trajectoryPoint.track_yaw = spline.calc_yaw(i);
            trajectoryPoint.track_curvature = spline.calc_curvature(i);
            trajectory_.push_back(trajectoryPoint);
        }
        return true;
    }

   bool Track_trackdrive::getTrajPath(Trajectory &TrajPath) {
       TrajPath = trajectory_;
   }

    bool Track_trackdrive::calTraj(Trajectory &relativePath) {
        if (trajectory_.empty()) {
            ROS_WARN("Trajectory is empty !");
            return false;
        }
        double vehicle_x = state_.vehicle_x;
        double vehicle_y = state_.vehicle_y;
        double vehicle_yaw = state_.vehicle_yaw;
        double vehicle_v = state_.vehicle_v;
        const double desired_velocity = param_.desire_vel;
        const int N = param_.N;
        const double dt = param_.dt;
        const double max_lat_acc = param_.max_lat_acc;
        TrajectoryPoint trajectoryPoint;
        relativePath.clear();
        for (int i = 0; i < trajectory_.size(); i++) {
            trajectoryPoint.track_x   = trajectory_[i].track_x;
            trajectoryPoint.track_y   = trajectory_[i].track_y;
            trajectoryPoint.track_yaw = trajectory_[i].track_yaw;
            trajectoryPoint.track_curvature = trajectory_[i].track_curvature;
            trajectoryPoint.track_v   = std::min(sqrt(param_.max_lat_acc / trajectoryPoint.track_curvature), desired_velocity);
            relativePath.push_back(trajectoryPoint);
        }
    }
}