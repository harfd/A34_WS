#pragma once
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "opencv2/imgproc.hpp"
#include "eigen3/Eigen/Core"
#include "ros/ros.h"
#include "type.hpp"
#include <cmath>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace ns_coordSystem
{
    using conePos = FSD::conePos;
    using pathPoint = FSD::pathPoint;
    class solver
    {
    public:
    void get_points(const sensor_msgs::PointCloud &center_points_,sensor_msgs::PointCloud &center_points_deal_,geometry_msgs::PoseArray &line_list_);
    void line_fitting(const geometry_msgs::PoseArray &line_list_,float &k,float &b, float &r);
    private:
    sensor_msgs::PointCloud fitting_points_; 
    };
    
}
