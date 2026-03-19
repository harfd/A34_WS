#include <ros/ros.h>
#include "coordSystem.hpp"
#include <sstream>

namespace ns_coordSystem {
// Constructor
CoordSystem::CoordSystem(ros::NodeHandle &nh) : nh_(nh) {
    nh_.param<bool>("check_imformation_imu",check_imformation_imu_,false);
    nh_.param<bool>("check_imformation_coordtrans",check_imformation_coordtrans_,false);
    nh_.param<float>("laser2base_length",laser2base_length_,2.6);
    nh_.param<int>("fitting_size",fitting_size_,6);
    nh_.param<std::string>("localization_way",localization_way_,"slam");
    nh_.param<std::string>("mission",mission_,"acceleration");
};

// Getters
sensor_msgs::PointCloud CoordSystem::getTfInfo() {
    return tf_info_;
}
std_msgs::Int8 CoordSystem::getErrorSignal() {
    return imu_error_;
}
geometry_msgs::Point CoordSystem::getImuCoord() {
    return imu_coord_;
}
std_msgs::String CoordSystem::getLocalizationWay() {
    way_.data = localization_way_;
    return way_;
};
std_msgs::String CoordSystem::getMission() {
    Mission_.data = mission_;
    return Mission_;
};
visualization_msgs::Marker CoordSystem::getTriangles() {
    return triangles_;
}
visualization_msgs::Marker CoordSystem::getPath() {
    return path_;
}

// Setters for subscriber
void CoordSystem::setImuState(coordSystem::imu_state msg) {
    imu_state_ = msg;
}
void CoordSystem::setCenterPoints(sensor_msgs::PointCloud msg) {
    center_points = msg;
}
void CoordSystem::setSlamState(const geometry_msgs::Pose2D msg) {
    slam_state_ = msg;
}

void CoordSystem::showImuState() {
    if (check_imformation_imu_) {
    std::cout << std::fixed <<"yaw_imu       : " << imu_state_.yaw<< std::endl;
    std::cout << std::fixed <<"N_vel_imu     : " << imu_state_.N_vel<< std::endl;
    std::cout << std::fixed <<"E_vel_imu     : " << imu_state_.E_vel<< std::endl;
    std::cout << std::fixed <<"vel_imu       : " << imu_state_.vel<< std::endl;
    std::cout << std::fixed <<"roll_acc_imu  : " << imu_state_.roll_acc<< std::endl;
    std::cout << std::fixed <<"pitch_acc_imu : " << imu_state_.pitch_acc<< std::endl;
    std::cout << std::fixed <<"yaw_acc_imu   : " << imu_state_.yaw_acc<< std::endl;
    std::cout << std::fixed <<"X_acc_imu     : " << imu_state_.X_acc<< std::endl;
    std::cout << std::fixed <<"Y_acc_imu     : " << imu_state_.Y_acc<< std::endl;
    std::cout << std::fixed <<"Z_acc_imu     : " << imu_state_.Z_acc<< std::endl;
    std::cout << std::fixed <<"longitude_imu : " << imu_state_.longitude<< std::endl;
    std::cout << std::fixed <<"latiude_imu   : " << imu_state_.latitude<< std::endl;
    }
    //due to my mistake, the longitude and latitude get reversed, it does not influence too much.Fix it in the follow code.
    // c.lpzt.lam = imu_state_.latitude;
    // c.lpzt.phi = imu_state_.longitude;
}

void CoordSystem::setInitialState() {
    int check_length = 5;
    if (!is_init && imu_state_.yaw!=0 && imu_state_.longitude!=0) {
        if (index < 100) {
            init_data_vec.push_back(imu_state_.longitude);
            if (index >= check_length && init_data_vec[index] == init_data_vec[index-check_length-1]) {
                ROS_INFO("Initialize IMU Posture Successed !");
                yaw_init_       = imu_state_.yaw;
                x_projection_init_ = x_projection;
                y_projection_init_ = y_projection;
                // std::cout << std::fixed << "x_projection : " << x_projection << std::endl;
                // std::cout << std::fixed << "y_projection : " << y_projection << std::endl;
                // std::cout << std::fixed << "x_projection_init_ : " << x_projection_init_ << std::endl;
                // std::cout << std::fixed << "y_projection_init_ : " << y_projection_init_ << std::endl;
                std::cout << std::fixed << "initial yaw angle : " << yaw_init_ << std::endl;
                is_init = true;
            }
            index += 1;
        }
    }
}

void CoordSystem::coordTrans() {

    const char *wgs84 = "+proj=longlat +datum=WGS84 +no_defs";
    const char *city = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=20500000 +y_0=0 +a=6378140 +b=6356755.288157528 +units=m +no_defs";

    PJ *P;
	PJ_COORD c, c_out;
    //transfer the coordinate from "wgs84" to the "projection" coordinate system
    //the longitude and latitude get reversed
	c.lpzt.lam = imu_state_.latitude;
    c.lpzt.phi = imu_state_.longitude;

	P = proj_create_crs_to_crs(PJ_DEFAULT_CTX,wgs84,city,NULL);
	c_out = proj_trans(P,PJ_FWD,c);

	x_projection = c_out.xy.x;
	y_projection = c_out.xy.y;
    
    setInitialState();
    double x_trans = x_projection - x_projection_init_;
    double y_trans = -(y_projection - y_projection_init_);
    double yaw_trans = imu_state_.yaw - yaw_init_;
    X_trans = x_trans * cos(-yaw_init_*M_PI/180.0) + y_trans * sin(-yaw_init_*M_PI/180.0);
    Y_trans = y_trans * cos(-yaw_init_*M_PI/180.0) - x_trans * sin(-yaw_init_*M_PI/180.0);
    // std::cout << std::fixed << "X_trans : " << X_trans << std::endl;
    // std::cout << std::fixed << "Y_trans : " << Y_trans << std::endl;
    // std::cout << std::fixed << "x_trans : " << x_trans << std::endl;
    // std::cout << std::fixed << "y_trans : " << y_trans << std::endl;
    // std::cout << std::fixed << "yaw_init : " << yaw_init_ << std::endl;
    
    imu_coord_.x = X_trans;
    imu_coord_.y = Y_trans;

    if (check_imformation_coordtrans_) {
    std::cout << std::fixed << "x_projection : " << x_projection << std::endl;
    std::cout << std::fixed << "y_projection : " << y_projection << std::endl;
    }

}

void CoordSystem::chooseMidPoints() {
    solver_.get_points(center_points,center_points_deal,line_list_);
    visualTriangles(line_list_,triangles_,path_);
}

void CoordSystem::line_fitting_init() {
    if (line_k == 0 && line_b == 0 && line_list_.poses.size() == fitting_size_) {
        ROS_WARN("Start initialized!");
        solver_.line_fitting(line_list_, line_k, line_b, line_correlation_coefficient);
    }
    else if (line_k != 0 && line_b !=0 && is_init_line == false){
        initialX = (center_points.points[0].x + center_points.points[1].x)/2;
        initialY = (center_points.points[0].y + center_points.points[1].y)/2;
        is_init_line = true;
        ROS_INFO("Line fitting successed!----->the line_k = %.2f the line_b = %.2f",line_k,line_b);
        ROS_INFO("Line fitting successed!----->the initialX = %.2f the initialY = %.2f",initialX,initialY);
    }
    line_list_.poses.clear();
}

void CoordSystem::tf_skeleton() {
    if (mission_ != "trackdrive") {
        if (line_k != 0 && line_b != 0) {
            // laser_link to base_link
            laser_link2base_link.x = -laser2base_length_;
            laser_link2base_link.y = 0;
            laser_link2base_link.z = 0;//replace z as yaw
            
            //slam_map to laser_link
            slam_map2laser_link.x = slam_state_.x;
            slam_map2laser_link.y = slam_state_.y;
            // double delta_yaw = -(imu_state_.yaw - yaw_init_)*M_PI/180;
            // while ((delta_yaw) >= M_PI)
            //     delta_yaw -= M_PI * 2.0;
            // while ((delta_yaw) <= -1.0 * M_PI)
            //     delta_yaw += M_PI * 2.0;
            // slam_map2laser_link.z = delta_yaw;

            double delta_yaw = slam_state_.theta;
            // std::cout << "delta_yaw : " << delta_yaw << std::endl;

            //slam_map to fssim_map
            slam_map2map.x = initialX;
            slam_map2map.y = initialY;
            slam_map2map.z = atan(line_k);

            //imu_map to imu_link
            imu_map2imu_link.x = X_trans;
            imu_map2imu_link.y = Y_trans;
            imu_map2imu_link.z = delta_yaw;

            tf_info_.points.clear();
            tf_info_.points.push_back(laser_link2base_link);
            tf_info_.points.push_back(slam_map2laser_link);
            tf_info_.points.push_back(slam_map2map);
            tf_info_.points.push_back(imu_map2imu_link);
        }
    }
    else {
        // laser_link to base_link
        laser_link2base_link.x = -laser2base_length_;
        laser_link2base_link.y = 0;
        laser_link2base_link.z = 0;

        slam_map2laser_link.x = 0;
        slam_map2laser_link.y = 0;
        double delta_yaw = -(imu_state_.yaw - yaw_init_)*M_PI/180;
        while ((delta_yaw) >= M_PI)
            delta_yaw -= M_PI * 2.0;
        while ((delta_yaw) <= -1.0 * M_PI)
            delta_yaw += M_PI * 2.0;
        slam_map2laser_link.z = delta_yaw;

        tf_info_.points.clear();
        tf_info_.points.push_back(laser_link2base_link);
        tf_info_.points.push_back(slam_map2laser_link);
    }
}

void CoordSystem::clearimuInfo() {
    imu_state_.yaw = 0;
    imu_state_.X_acc = 0;
    imu_state_.Y_acc = 0;
}
void CoordSystem::checkImuError() {
    if (imu_state_.X_acc==0 && imu_state_.Y_acc==0 && imu_state_.yaw==0)
        imu_error_.data = 1;
    else
        imu_error_.data = 0;
}

void CoordSystem::runAlgorithm() {

    // CoordSystem::showImuState();
    // CoordSystem::coordTrans();
    CoordSystem::chooseMidPoints();
    CoordSystem::line_fitting_init();
    CoordSystem::tf_skeleton();
    if (mission_ == "check") {
        checkImuError();
        clearimuInfo();
    }
}

}
