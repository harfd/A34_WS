#include"utility.h"


class ConversionPosion
{
public:
    
    ConversionPosion()
    {   
        SubOdometry = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init",5,&ConversionPosion::OdometryHandle,this);
        pubOdometry = nh.advertise<geometry_msgs::Pose2D>("/my_position",5);
    }

    void OdometryHandle(const nav_msgs::Odometry::ConstPtr &msg)
    {
        currentHeader = msg->header;
        
        double roll,pitch,yaw;

        geometry_msgs::Quaternion geoQut = msg->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQut.z, -geoQut.x, -geoQut.y, geoQut.w)).getRPY(roll,pitch,yaw);

        My_Position.theta = (yaw/PI)*180;
        My_Position.x = msg->pose.pose.position.x;
        My_Position.y = msg->pose.pose.position.y;

        pubOdometry.publish(My_Position);

    }

    // void OdometryHandle(const nav_msgs::Odometry::ConstPtr &msg)
    // {
    //     geometry_msgs::PoseStamped thispose;
    //     thispose.header.frame_id = msg->header.frame_id;
    //     thispose.header.stamp = ros::Time::now();
    //     thispose.pose.position.x = msg->pose.pose.position.x;
    //     thispose.pose.position.y = msg->pose.pose.position.y;
    //     thispose.pose.position.z = msg->pose.pose.position.z;
    //     thispose.pose.orientation = msg->pose.pose.orientation;

    //     path.poses.push_back(thispose);
    //     path.header.frame_id = msg->header.frame_id;
    //     path.header.stamp = ros::Time::now();

    //     pubOdometry.publish(path);

    // }
    
    

private:
    ros::NodeHandle nh;
    ros::Subscriber SubOdometry;
    ros::Publisher pubOdometry;
    
    std_msgs::Header currentHeader;

    geometry_msgs::Pose2D My_Position;
    nav_msgs::Path path;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lego_loam");
    
    ConversionPosion Cp;

    ROS_INFO("\033[1;32m---->\033[0m  ConversionPosion Started.");

    ros::spin();
    
    return 0;
}