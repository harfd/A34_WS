#include "Solver/solver_purePursuit.h"

namespace ns_control {
    void Solver_purePursuit::solve() {
        double desire_vel = param_.desire_vel;
        double car_length = param_.car_length;
        int i_next = param_.look_ahead;
        { // Steering Control
        const double beta_est = controlCommand_.steering_angle.data * 0.5;
        next_point.x                            = trajectory_[i_next].track_x*std::cos(state_.vehicle_yaw) 
                                                - trajectory_[i_next].track_y*std::sin(state_.vehicle_yaw);
        next_point.y                            = trajectory_[i_next].track_x*std::sin(state_.vehicle_yaw) 
                                                + trajectory_[i_next].track_y*std::cos(state_.vehicle_yaw);
        //transform the coordinate form "/base_link" to the "/map"
        const double eta                        = std::atan2(next_point.y, next_point.x)
                                                    - (state_.vehicle_yaw + beta_est);
        const double length                     = std::hypot(next_point.y, next_point.x);

        controlCommand_.steering_angle.data    = static_cast<float>(std::atan(2.0*car_length / length * std::sin(eta)));
        if (controlCommand_.steering_angle.data > 0.5) 
            controlCommand_.steering_angle.data = 0.5;
        else if (controlCommand_.steering_angle.data < -0.5)
            controlCommand_.steering_angle.data = -0.5;
        else
            controlCommand_.steering_angle.data = controlCommand_.steering_angle.data;
        
        }

        { // Speed Controller
        const double vel = state_.vehicle_v;
        controlCommand_.throttle.data = static_cast<float>(desire_vel - vel);
        if (controlCommand_.throttle.data > 1) 
            controlCommand_.throttle.data = 1;
        else if (controlCommand_.throttle.data < -1)
            controlCommand_.throttle.data = -1;
        else
            controlCommand_.throttle.data = controlCommand_.throttle.data;

        }

        std::cout << "steering:  " << controlCommand_.steering_angle.data << std::endl;
        std::cout << "throttle:  " << controlCommand_.throttle.data << std::endl;
    }
}