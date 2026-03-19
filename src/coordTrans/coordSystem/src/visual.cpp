#include "visual.hpp"

namespace ns_coordSystem {
    void visualTriangles(geometry_msgs::PoseArray line_list_,visualization_msgs::Marker &visualTriangles,visualization_msgs::Marker &visualPath) {
        visualTriangles.points.clear();
		visualization_msgs::Marker line_list;
		line_list.header.frame_id = "laser_link";
		line_list.header.stamp = ros::Time();
		line_list.ns = "points_and_lines";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = 0.1;
		line_list.color.g = 1.0;
		line_list.color.a = 1.0;

		geometry_msgs::Point p1;

        for (int i = 0; i < line_list_.poses.size(); i++) {
            p1.x = line_list_.poses[i].position.x;
            p1.y = line_list_.poses[i].position.y;
            p1.z = 0;
            line_list.points.push_back(p1);
        }

		visualTriangles = line_list;

		visualPath.points.clear();
		visualization_msgs::Marker path_list;
		path_list.header.frame_id = "laser_link";
		path_list.header.stamp = ros::Time();
		path_list.ns = "path";
		path_list.action = visualization_msgs::Marker::ADD;
		path_list.type = visualization_msgs::Marker::LINE_STRIP;
		path_list.scale.x = 0.1;
		path_list.color.r = 1.0;
		path_list.color.a = 1.0;

		geometry_msgs::Point tmp;

		for (int i = 0; i < line_list_.poses.size()/2; i++) {
			tmp.x = (line_list_.poses[2*i].position.x + line_list_.poses[2*i+1].position.x)/2;
			tmp.y = (line_list_.poses[2*i].position.y + line_list_.poses[2*i+1].position.y)/2;
			tmp.z = 0;
			path_list.points.push_back(tmp);
		}

		visualPath = path_list;
    }

};