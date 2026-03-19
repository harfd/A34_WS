#pragma once

#include "Basic/types.h"
#include "Basic/param.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "geometry_msgs/Point.h"


namespace ns_control
{
    class Solver
    {
    public:
    void setSolverState(const VehicleState &state);
    void setTrajectory(const Trajectory &trajectory);
    fsd_common_msgs::ControlCommand getControlCommand();
    Trajectory getTrajectory();
    virtual void solve() = 0;

    protected:
    Trajectory trajectory_;
    Trajectory predictive_path;
    VehicleState state_;
    fsd_common_msgs::ControlCommand controlCommand_;
    };
    
}
