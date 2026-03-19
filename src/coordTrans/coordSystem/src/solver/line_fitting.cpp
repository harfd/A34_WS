#include "solver/line_fitting.h"

namespace ns_coordSystem
{   
    void solver::get_points(const sensor_msgs::PointCloud &center_points_,sensor_msgs::PointCloud &center_points_deal_,geometry_msgs::PoseArray &line_list_) {
        cv::Rect rect(-200, -200, 400, 400);
        cv::Subdiv2D coneSet(rect);
        for (int i = 0; i < center_points_.points.size(); i++) {
            cv::Point2f fp;
            fp.x = center_points_.points[i].x;
            fp.y = center_points_.points[i].y;
            coneSet.insert(fp);
        }
        std::vector<cv::Vec4f> edges;
        coneSet.getEdgeList(edges);
        cv::Point2f outer_vtx[3];
        for (int i = 0; i < 3; i++)
            outer_vtx[i] = coneSet.getVertex(i + 1);

        std::vector<pathPoint> midSet;
        for (int i = 0; i < edges.size(); i++) {
            if (edges[i][0] == outer_vtx[0].x && edges[i][1] == outer_vtx[0].y ||
                edges[i][0] == outer_vtx[1].x && edges[i][1] == outer_vtx[1].y ||
                edges[i][0] == outer_vtx[2].x && edges[i][1] == outer_vtx[2].y ||
                edges[i][2] == outer_vtx[0].x && edges[i][3] == outer_vtx[0].y ||
                edges[i][2] == outer_vtx[1].x && edges[i][3] == outer_vtx[1].y ||
                edges[i][2] == outer_vtx[2].x && edges[i][3] == outer_vtx[2].y ||
                fabs(edges[i][0] - edges[i][2]) > 0.5 || fabs(edges[i][1] + edges[i][3]) > 1) // filter the cone while Y coordinate deviate too much
                continue;

            pathPoint tmp;
            tmp.left_cone = conePos(edges[i][0],edges[i][1]);
            tmp.right_cone = conePos(edges[i][2],edges[i][3]);
            tmp.id = i;
            tmp.calMidCone();
            midSet.push_back(tmp);
        }
        // std::cout << "midSet.size() : "  << midSet.size() << std::endl;
        if (midSet.size() != 0) {
            sort(midSet.begin(),midSet.end(),
                            [&](const pathPoint &a,const pathPoint &b) {
                                return (std::hypot(a.x,a.y)<std::hypot(b.x,b.y));
                                            });
        }
        for (int i = 0; i < midSet.size(); i++ ) {
            // std::cout << midSet[i].x << "  " << midSet[i].y << std::endl;
            geometry_msgs::Pose p1, p2;
            p1.position.x = midSet[i].left_cone.x;
            p1.position.y = midSet[i].left_cone.y;
            p2.position.x = midSet[i].right_cone.x;
            p2.position.y = midSet[i].right_cone.y;
            line_list_.poses.push_back(p1);
            line_list_.poses.push_back(p2);
        }

    }

    
    void solver::line_fitting(const geometry_msgs::PoseArray &line_list_,float &k,float &b, float &r) {


        int center_points_num = line_list_.poses.size();
        int fitting_points_num = center_points_num/2;
        for (int i = 0; i < fitting_points_num; i++) {
            geometry_msgs::Point32 tmp;
            tmp.x = (line_list_.poses[2*i].position.x + line_list_.poses[2*i+1].position.x)/2;
            tmp.y = (line_list_.poses[2*i].position.y + line_list_.poses[2*i+1].position.y)/2;
            tmp.z = 0;
            fitting_points_.points.push_back(tmp);
        }

        float A = 0.0;  
        float B = 0.0;  
        float C = 0.0;  
        float D = 0.0;  
        float E = 0.0;  
        float F = 0.0; 

        for (int i = 0; i < fitting_points_num; i++) {
            A += (fitting_points_.points[i].x) * (fitting_points_.points[i].x);
            B += (fitting_points_.points[i].x);
            C += (fitting_points_.points[i].x) * (fitting_points_.points[i].y);
            D += (fitting_points_.points[i].y);
        }

        float temp = 0.0;
        if (temp = fitting_points_num*A - B*B) {
            k = (fitting_points_num*C - B*D)/temp;
            b = (A*D - B*C)/temp;
        }
        else {
            ROS_WARN("Initialize line fitting false!");
        }

        float X_mean, Y_mean;
        X_mean = B/fitting_points_num;
        Y_mean = D/fitting_points_num;

        float tempSumXX = 0.0, tempSumYY = 0.0;  
        for (int i = 0; i < fitting_points_num; i++) {  
        tempSumXX += (fitting_points_.points[i].x - X_mean) * (fitting_points_.points[i].x - X_mean);  
        tempSumYY += (fitting_points_.points[i].y - Y_mean) * (fitting_points_.points[i].y - Y_mean);  
        E += (fitting_points_.points[i].x - X_mean) * (fitting_points_.points[i].y - Y_mean);  
        }  
        F = sqrt(tempSumXX) * sqrt(tempSumYY);
        r = E / F;
        // std::cout << "k :" << k << std::endl;
        // std::cout << "b :" << b << std::endl;
        // std::cout << "r :" << r << std::endl;
    }
}
