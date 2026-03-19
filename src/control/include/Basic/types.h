#ifndef TYPE_H
#define TYPE_H

#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"

namespace ns_control{
    struct VehicleState{
        double vehicle_x;
        double vehicle_y;
        double vehicle_yaw;
        double vehicle_v;
        double steering_angle;
        double throttle;
        VehicleState(fsd_common_msgs::CarState state, fsd_common_msgs::ControlCommand cmd){
            vehicle_x = state.car_state.x;
            vehicle_y = state.car_state.y;
            vehicle_yaw = state.car_state.theta;
            vehicle_v = std::hypot(state.car_state_dt.car_state_dt.x, state.car_state_dt.car_state_dt.y);
            steering_angle = cmd.steering_angle.data;
            throttle = cmd.throttle.data;
        }
        VehicleState() {
        }
    };

    struct TrajectoryPoint{
        double track_x;
        double track_y;
        double track_yaw;
        double track_v;
        double track_curvature;
    };
    typedef std::vector<TrajectoryPoint> Trajectory;
}
# endif