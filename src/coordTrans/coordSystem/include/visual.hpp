#ifndef VISUAL_PATH_HPP
#define VISUAL_PATH_HPP

#include "type.hpp"
#include <string>
#include "opencv2/imgproc.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"

namespace ns_coordSystem{
    void visualTriangles(geometry_msgs::PoseArray line_list_,visualization_msgs::Marker &visualTriangles,visualization_msgs::Marker &visualPath);
};

#endif