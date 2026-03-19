/*
version:    1.0
author:     Shawn Pan 
Time:       2021.10.17 18:12
雷达断线检测
断线2s后输出topic并退出节点
pub: /error_signal(std_msgs::Int8 data=2)
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <time.h>

bool is_lidar_ok_ = 0;
time_t start_time;
time_t check_time;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msgs){
    ROS_INFO("lidar msgs ok!");
    is_lidar_ok_ = 1;
    start_time = check_time;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"lidar_check");
    ros::NodeHandle nh("~");
    ros::Subscriber lidar_sub;
    ros::Publisher error_signal_pub;
    ros::Rate loop_rate(30);
    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 10, &lidar_callback);
    error_signal_pub = nh.advertise<std_msgs::Int8>("/error_signal",1);
    time(&start_time);
    while (nh.ok())
    {
        time(&check_time);
        if ( (check_time-start_time)>=2){
            ROS_INFO("no lidar msgs!");
            std_msgs::Int8 error_msgs;
            error_msgs.data = 2;
            error_signal_pub.publish(error_msgs);
            ros::spinOnce();
            loop_rate.sleep();
            return 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }    
    return 0;
}


