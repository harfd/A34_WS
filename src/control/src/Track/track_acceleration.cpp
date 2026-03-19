#include "Track/track_acceleration.h"
#include "ros/ros.h"

namespace ns_control
{
     bool Track_acceleration::getTraj(){

        const double interval = param_.interval;
        const double desire_vel = param_.desire_vel;
        double distance = std::hypot(endPoint_.x, endPoint_.y);
        double gradient = endPoint_.y / endPoint_.x;
        TrajectoryPoint trajectoryPoint;
        trajectory_.clear();
        for (double i = 0; i < distance; i += interval) {
        trajectoryPoint.track_x = i / std::hypot(1.0, gradient);
        trajectoryPoint.track_y = trajectoryPoint.track_x * gradient;
        trajectoryPoint.track_yaw = atan2(trajectoryPoint.track_y, trajectoryPoint.track_x);
        trajectoryPoint.track_v = desire_vel;
        trajectoryPoint.track_curvature = 0;
        trajectory_.push_back(trajectoryPoint);
        }
        return true;
   }

   bool Track_acceleration::getTrajPath(Trajectory &TrajPath) {
       TrajPath = trajectory_;
   }

    bool Track_acceleration::calTraj(Trajectory &relativePath){ 
        if (trajectory_.empty()) {
        ROS_WARN("Trajectory is empty !");};

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

        int index_min = -1;
        double min = 777;

        for (int i = 0; i < trajectory_.size(); i++) {
            double delta_x = trajectory_[i].track_x - vehicle_x;
            double delta_y = trajectory_[i].track_y - vehicle_y;
            double dist = std::hypot(delta_x, delta_y);
            if (dist < min) {
            min = dist;
            index_min = i;
            }
        }
        //get the closest point tag(index_min) from the vehicle to to the track
        if (!is_init_ && ros::Time::now().toSec()!=0) {
            lateralDisVecFile_.open("/home/rzs/data_record/dataFile(acceleration).txt", std::ofstream::app);
            std::ofstream file_writer("/home/rzs/data_record/dataFile(acceleration).txt", std::ios_base::out);
            time_init_ = ros::Time::now().toSec();
            is_init_ = true;
        }
        else if (is_init_ && ros::Time::now().toSec()!=0) {
            lateralDis_ = std::hypot((vehicle_x - trajectory_[index_min].track_x),
                                     (vehicle_y - trajectory_[index_min].track_y));
            lateralDisVecFile_ << ros::Time::now().toSec() - time_init_ << "  " << lateralDis_ << std::endl;
        }
        
        double distancePredict;
        for (int i = 0; i < N; i++) {
            if (i != 0)
                distancePredict += vehicle_v * dt;

            int index = int(distancePredict / param_.interval);
            int next = index + index_min > trajectory_.size() ? trajectory_.size() - 1 : index + index_min;
            double delta_x = trajectory_[next].track_x - vehicle_x;
            double delta_y = trajectory_[next].track_y - vehicle_y;
            double delta_yaw = trajectory_[next].track_yaw - vehicle_yaw;

            while ((delta_yaw) >= M_PI)
            delta_yaw -= M_PI * 2.0;
            while ((delta_yaw) <= -1.0 * M_PI)
            delta_yaw += M_PI * 2.0;

            double temp_x, temp_y;
            temp_x = delta_x * cos(vehicle_yaw) + delta_y * sin(vehicle_yaw);
            temp_y = delta_y * cos(vehicle_yaw) - delta_x * sin(vehicle_yaw);
            //transform the coordinate from "/map" to the "/base_link"

            TrajectoryPoint trajectoryPoint;
            trajectoryPoint.track_x = temp_x;
            trajectoryPoint.track_y = temp_y;
            trajectoryPoint.track_yaw = delta_yaw;
            trajectoryPoint.track_curvature = fabs(trajectory_[next].track_curvature);
            trajectoryPoint.track_v = std::min(sqrt(param_.max_lat_acc / trajectoryPoint.track_curvature), desired_velocity);
            relativePath.push_back(trajectoryPoint);
        }
    }
}