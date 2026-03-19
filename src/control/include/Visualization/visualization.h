#pragma once
#include "fsd_common_msgs/Map.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "Basic/param.h"
#include "Basic/types.h"

namespace ns_control
{
    void visualize_trajectory(const Trajectory &trajectory   , visualization_msgs::MarkerArray &visual, 
                              const std::string &frame       , const std::vector<float> &color,
                              const std_msgs::Header &header , bool is_vel, int i_interval);
    void visualize_map(const fsd_common_msgs::Map &map       , visualization_msgs::MarkerArray &visual);
    void color_map(double vel, std::vector<float> &color);
    void visualize_traj(const Trajectory &trajectory, visualization_msgs::MarkerArray &visualTraj);
}
