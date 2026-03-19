#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include "Basic/types.h"
#include "Basic/param.h"
#include "Track/track_base.h"
#include "Track/track_acceleration.h"
#include "Track/track_skidpad.h"
#include "Track/track_trackdrive.h"
#include "Solver/solver_base.h"
#include "Solver/solver_purePursuit.h"
#include "Solver/solver_mpc.h"
#include "Visualization/visualization.h"

namespace ns_control {

class Control {

 public:
  // Constructor
  Control(ros::NodeHandle& nh);

	// Getters for calculated imformation for publisher
  fsd_common_msgs::ControlCommand getControlCommand();
  visualization_msgs::MarkerArray getTrajectoryPath();
  visualization_msgs::MarkerArray getTargetPath();
  visualization_msgs::MarkerArray getPredictPath();

	// Setters for callback functions of subscriber
  void setCarState(const fsd_common_msgs::CarState &msg);
  void setPlanningTraj(const geometry_msgs::PoseArray &msg);
  void setEndPoint(const geometry_msgs::Point &msg);
  bool check();
  void setTrack();
  void runAlgorithm();
  
private:

	ros::NodeHandle& nh_;
  std::string mission_;
  std::string controller_;
	
  Trajectory trajectory_;
  Track *track_;
  Track_acceleration  track_acceleration_;
  Track_skidpad track_skidpad_;
  Track_trackdrive track_trackdrive_;
  Solver *solver_;
  Solver_purePursuit  solver_purePursuit_;
  Solver_mpc  solver_mpc_;

  fsd_common_msgs::ControlCommand controlCommand_;
  fsd_common_msgs::CarState carState_;
  geometry_msgs::PoseArray planningTraj_;
  geometry_msgs::Point endPoint_;

  Trajectory relativePath_;
  visualization_msgs::MarkerArray trajectoryPath_;
  visualization_msgs::MarkerArray targetPath_;
  visualization_msgs::MarkerArray predictPath_;
  visualization_msgs::MarkerArray visualTraj_;

  double acceleration_length_;
  bool is_init = false;
};
}

#endif //CONTROL_HPP
