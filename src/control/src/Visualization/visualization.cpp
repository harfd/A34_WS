#include "Visualization/visualization.h"

namespace ns_control
{
    void visualize_trajectory(const Trajectory &trajectory   , visualization_msgs::MarkerArray &visual, 
                              const std::string &frame       , const std::vector<float> &color,
                              const std_msgs::Header &header , bool is_vel, int i_interval) {
        visual.markers.clear();
        std::vector<float> color_tmp;
        visualization_msgs::Marker tmp;
        tmp.header.frame_id = frame;
        tmp.header.stamp = ros::Time();
        tmp.ns = "trajectory";
        tmp.action = visualization_msgs::Marker::ADD;
        tmp.type = visualization_msgs::Marker::LINE_STRIP;
        tmp.scale.x = 0.1;
        color_tmp = color;
        tmp.color.r = color_tmp[0];
        tmp.color.g = color_tmp[1];
        tmp.color.b = color_tmp[2];
        tmp.color.a = 1.0;
        geometry_msgs::Point p;
            for (const auto &iter:trajectory) {
                p.x = iter.track_x;
                p.y = iter.track_y;
                tmp.points.push_back(p);
            }
        visual.markers.push_back(tmp);
    }

    void visualize_map(const fsd_common_msgs::Map &map, visualization_msgs::MarkerArray &visual) {
        visual.markers.clear();
        for (int i = 0; i < map.cone_red.size(); i++) {
            visualization_msgs::Marker tmp;
            tmp.header.frame_id = "/map";
            tmp.header.stamp = ros::Time::now();
            tmp.ns = "red_" + std::to_string(i);
            tmp.action = visualization_msgs::Marker::ADD;
            tmp.type = visualization_msgs::Marker::POINTS;
            tmp.scale.x = 0.8;
            tmp.scale.y = 0.8;
            tmp.scale.z = 0.8;

            tmp.color.r = 1;
            tmp.color.g = 0;
            tmp.color.b = 0;
            tmp.color.a = 1.0;
            geometry_msgs::Point p;
            p.x = map.cone_red[i].position.x;
            p.y = map.cone_red[i].position.y;
            tmp.points.push_back(p);
            visual.markers.push_back(tmp);
        }

        for (int i = 0; i < map.cone_blue.size(); i++) {
            visualization_msgs::Marker tmp;
            tmp.header.frame_id = "/map";
            tmp.header.stamp = ros::Time::now();
            tmp.ns = "blue_" + std::to_string(i);
            tmp.action = visualization_msgs::Marker::ADD;
            tmp.type = visualization_msgs::Marker::POINTS;
            tmp.scale.x = 0.8;
            tmp.scale.y = 0.8;
            tmp.scale.z = 0.8;

            tmp.color.r = 0;
            tmp.color.g = 0;
            tmp.color.b = 1;
            tmp.color.a = 1.0;
            geometry_msgs::Point p;
            p.x = map.cone_blue[i].position.x;
            p.y = map.cone_blue[i].position.y;
            tmp.points.push_back(p);
            visual.markers.push_back(tmp);
        }
    }

    void color_map(double vel, std::vector<float> &color) {
        double temp = vel / param_.desire_vel;
        if (temp > 1)
            temp = 1;
        if (temp < 0)
            temp = 0;
        color.clear();
        color.resize(3);
        color[0] = temp;
        color[1] = 0;
        color[2] = 1 - temp;
    }

    void visualize_traj(const Trajectory &trajectory, visualization_msgs::MarkerArray &visualTraj) {
        visualTraj.markers.clear();
        visualization_msgs::Marker TrajPoint;
        TrajPoint.header.frame_id = "/base_link";
        TrajPoint.header.stamp = ros::Time();
        TrajPoint.ns = "red";
        TrajPoint.action = visualization_msgs::Marker::ADD;
        TrajPoint.type = visualization_msgs::Marker::LINE_STRIP;
        TrajPoint.scale.x = 0.2;
        TrajPoint.color.r = 1.0;
        TrajPoint.color.a = 1.0;
        geometry_msgs::Point p;
        for(const auto &iter: trajectory) {
            p.x = iter.track_x;
            p.y = iter.track_y;
            TrajPoint.points.push_back(p);
        }
        visualTraj.markers.push_back(TrajPoint);
    }
}
