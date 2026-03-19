#include "Solver/solver_base.h"

namespace ns_control
{
    void Solver::setSolverState(const VehicleState &state) { state_ = state; }
    void Solver::setTrajectory(const Trajectory &trajectory) { trajectory_ = trajectory; }
    fsd_common_msgs::ControlCommand Solver::getControlCommand() { return controlCommand_; }
    Trajectory Solver::getTrajectory() { return predictive_path; }
}
