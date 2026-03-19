#include "Track/track_skidpad.h"

namespace ns_control
{
    bool Track_skidpad::getTraj(){
        const double car_length = param_.car_length;
        const double interval = param_.interval;
        const double forward_distance = param_.forward_distance;
        const double circle_radius = param_.circle_radius;
        const double right_circle_x = forward_distance;
        const double right_circle_y = -circle_radius;
        const double left_circle_x = right_circle_x;
        const double left_circle_y = circle_radius;

        TrajectoryPoint trajectoryPoint;
        trajectory_.clear();
 
        for (double i = -2.0; i < ( forward_distance); i += interval) {
            trajectoryPoint.track_x = i;
            trajectoryPoint.track_y = 0;
            trajectoryPoint.track_yaw = 0;
            trajectoryPoint.track_curvature = 0;
            trajectory_.push_back(trajectoryPoint);
        }


        //right_circle
        for (double i = 0; i < 4 * M_PI; i += interval / circle_radius) {
            trajectoryPoint.track_x = circle_radius * std::cos(90 * M_PI / 180 - i) + right_circle_x;
            trajectoryPoint.track_y = circle_radius * std::sin(90 * M_PI / 180 - i) + right_circle_y;
            trajectoryPoint.track_yaw = -i;
            trajectoryPoint.track_curvature = 1/circle_radius;
            trajectory_.push_back(trajectoryPoint);
        }

        //left_circle
        for (double i = 0; i < 4 * M_PI; i += interval / circle_radius) {
            trajectoryPoint.track_x = circle_radius * std::cos(-90 * M_PI / 180 + i) + left_circle_x;
            trajectoryPoint.track_y = circle_radius * std::sin(-90 * M_PI / 180 + i) + left_circle_y;
            trajectoryPoint.track_yaw = i;
            trajectoryPoint.track_curvature = 1/circle_radius;
            trajectory_.push_back(trajectoryPoint);
        }

        //line again
        for (float i = forward_distance; i < (forward_distance) + 30; i += interval) {
            trajectoryPoint.track_x = i;
            trajectoryPoint.track_y = 0;
            trajectoryPoint.track_yaw = 0;
            trajectoryPoint.track_curvature = 0;
            trajectory_.push_back(trajectoryPoint);
        }

        return true;
    }

    bool Track_skidpad::getTrajPath(Trajectory &TrajPath) {
       TrajPath = trajectory_;
   }

    bool Track_skidpad::calTraj(Trajectory &relativePath){
        double vehicle_x = state_.vehicle_x;
        double vehicle_y = state_.vehicle_y;
        double vehicle_yaw = state_.vehicle_yaw;
        double vehicle_v = state_.vehicle_v;
        const double desired_velocity = param_.desire_vel;
        const int N = param_.N;
        const double dt = param_.dt;
        TrajectoryPoint trajectoryPoint;
        relativePath.clear();

        int index_min = -1;
        double min = 777;

        if (vehicle_x != 0) {
            for (int i = now_state; i < trajectory_.size() && i < now_state + 100; i++) {
                double delta_x = trajectory_[i].track_x - vehicle_x;
                double delta_y = trajectory_[i].track_y - vehicle_y;
                double dist = std::hypot(delta_x, delta_y);
                if (dist < min) {
                min = dist;
                index_min = i;
                }
            }
        }
        else 
            index_min = 0;
        now_state = index_min;
        if (!is_init_ && ros::Time::now().toSec()!=0) {
            lateralDisVecFile_.open("/home/rzs/data_record/dataFile(skidpad).txt", std::ofstream::app);
            std::ofstream file_writer("/home/rzs/data_record/dataFile(skidpad).txt", std::ios_base::out);
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
            //transform the coordinate from "/fssim_map" to the "/base_link"

            TrajectoryPoint trajectoryPoint;
            trajectoryPoint.track_x = temp_x;
            trajectoryPoint.track_y = temp_y;
            trajectoryPoint.track_yaw = delta_yaw;
            trajectoryPoint.track_curvature = fabs(trajectory_[next].track_curvature);
            trajectoryPoint.track_v =  desired_velocity;
            relativePath.push_back(trajectoryPoint);
        }
    }
}
