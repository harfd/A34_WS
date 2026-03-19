#include "Track/track_base.h"

Param param_;
namespace ns_control{
    void Track::setTrackEndPoint(const geometry_msgs::Point &endPoint) {
        endPoint_ = endPoint;
    }
    void Track::setPlanningPath(geometry_msgs::PoseArray &planningTraj) {
        planningTraj_ = planningTraj;
    }
    void Track::setTrackVehicleState(const VehicleState &state) {
        state_ = state;
    }
}
