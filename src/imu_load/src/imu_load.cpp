#include <ros/ros.h>
#include "imu_load.hpp"
#include <sstream>
#include<sensor_msgs/Imu.h>

namespace ns_imu_load {

serial_imu_load::serial_imu_load(ros::NodeHandle &nh) : nh_(nh) {
    if (!nh_.param<std::string>("serial_port",serial_port_,"/dev/ttyUSB0")){
        ROS_WARN_STREAM("Did not load serial_port. Standard value is: " << serial_port_);
    }
    if (!nh_.param<std::string>("imu_msgs_topic_name",imu_msgs_topic_name_,"/imu/data")){
        ROS_WARN_STREAM("Did not load imu_msgs_topic_name. Standard value is: " << imu_msgs_topic_name_);
    }
    if (!nh_.param("baudrate",baudrate_,115200)){
        ROS_WARN_STREAM("Did not load baudrate. Standard value is: " << baudrate_);
    }
    if (!nh_.param("node_rate",node_rate_,50)){
        ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
    };
    publishToTopics();
}



int serial_imu_load::serial_init() {
    try { 
        ser.setPort(serial_port_); 
        ser.setBaudrate(baudrate_); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open();
   }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port "); 
            return -1;     
    } 
    if (ser.isOpen()){
        ROS_INFO_STREAM("The port initialize succeed.");
        return 0;
    }
    else
        return -1;

}

void serial_imu_load::getAndCheck(){
    ros::Rate loop_rate(400); 
    while(ros::ok){
        if(ser.available()){
        int n = ser.available();
        std::cout << "The size : " << std::dec << n << std::endl;
        uint8_t buffer[512];
        ser.read(buffer,n);
        int index;
        for (int i = 0; i < n; i++){
            if (buffer[i] == 0xbd && buffer[i+1] == 0xdb && buffer[i+2] == 0x0b && (n-i)>57){
                index = i;
                short Yaw = solver_byte2(index+7,2,buffer);
                short N_Vel = solver_byte2(index+33,2,buffer);
                short E_Vel = solver_byte2(index+35,2,buffer);
                short roll_acc = solver_byte2(index+9,2,buffer);
                short pitch_acc = solver_byte2(index+11,2,buffer);
                short yaw_acc = solver_byte2(index+13,2,buffer);
                short X_Acc = solver_byte2(index+15,2,buffer);
                short Y_Acc = solver_byte2(index+17,2,buffer);
                short Z_Acc = solver_byte2(index+19,2,buffer);
                int Longitude = solver_byte4(index+21,4,buffer);
                int Latitude = solver_byte4(index+25,4,buffer);
                int Height = solver_byte4(index+29,4,buffer);

                float Yaw_f = Yaw;
                float N_Vel_f = N_Vel;
                float E_Vel_f = E_Vel;
                float roll_acc_f = roll_acc;
                float pitch_acc_f = pitch_acc;
                float yaw_acc_f = yaw_acc;
                float X_Acc_f = X_Acc;
                float Y_Acc_f = Y_Acc;
                float Z_Acc_f = Z_Acc;
                float Longitude_f = Longitude;
                float Latitude_f = Latitude;
                float Height_f = Height;


                float yaw_imu = Yaw_f*(1.0*360/32768);
                float N_vel_imu = N_Vel_f*float(1e2/32768);
                float E_vel_imu = E_Vel_f*float(1e2/32768);
                float roll_acc_imu = roll_acc_f*float(1.0*300*3.14/32768/180);
                float pitch_acc_imu = -pitch_acc_f*float(1.0*300*3.14/32768/180);
                float yaw_acc_imu = -yaw_acc_f*float(1.0*300*3.14/32768/180);
                float X_acc_imu = X_Acc_f*float(1.0*12*9.8/32768);
                float Y_acc_imu = -Y_Acc_f*float(1.0*12*9.8/32768);
                float Z_acc_imu = -Z_Acc_f*float(1.0*12*9.8/32768);
                float longitude_imu = Longitude_f*float(1e-7);
                float latitude_imu = Latitude_f*float(1e-7);
                float vel_imu = std::hypot(N_vel_imu,E_vel_imu);
                float height_imu = Height_f*float(1e-3);

                std::cout << "yaw_imu : " << yaw_imu << std::endl;
                std::cout << "N_vel_imu : " << N_vel_imu << std::endl;
                std::cout << "E_vel_imu : " << E_vel_imu << std::endl;
                std::cout << "vel_imu : " << vel_imu << std::endl;
                std::cout << "roll_acc_imu : " << roll_acc_imu << std::endl;
                std::cout << "pitch_acc_imu : " << pitch_acc_imu << std::endl;
                std::cout << "yaw_acc_imu : " << yaw_acc_imu << std::endl;
                std::cout << std::fixed << "X_acc_imu : " << X_acc_imu << std::endl;
                std::cout << "Y_acc_imu : " << Y_acc_imu << std::endl;
                std::cout << "Z_acc_imu : " << Z_acc_imu << std::endl;
                std::cout << "longitude_imu : " << longitude_imu << std::endl;
                std::cout << "laitude_imu : " << latitude_imu << std::endl;
                std::cout << "height_imu : " << height_imu << std::endl;
                ROS_INFO_STREAM("Read the imu imformation!");

               /* imu_state_.yaw = yaw_imu;
                imu_state_.N_vel = N_vel_imu;
                imu_state_.E_vel = E_vel_imu;
                imu_state_.vel = vel_imu;  */
                imu_state_.header.frame_id = "/imu";
                imu_state_.header.stamp = ros::Time::now();
                imu_state_.angular_velocity.x = roll_acc_imu;
                imu_state_.angular_velocity.y = pitch_acc_imu;
                imu_state_.angular_velocity.z = yaw_acc_imu;
                imu_state_.linear_acceleration.x = X_acc_imu;
                imu_state_.linear_acceleration.y = Y_acc_imu;
                imu_state_.linear_acceleration.z = Z_acc_imu;
                imu_state_.orientation_covariance = {-1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0};
                imu_state_.angular_velocity_covariance ={ 0.0004,0,0,0,0.0004,0,0,0,0.0004};
                imu_state_.linear_acceleration_covariance = {0.0004,0,0,0,0.0004,0,0,0,0.0004};
                
             //   imu_state_.longitude = longitude_imu;
            //    imu_state_.latitude = latitude_imu;
                
                }
                sendMsg();
            }
        } 
    }
    loop_rate.sleep();
}

sensor_msgs::Imu serial_imu_load::getImuState(){
    return imu_state_;}
void serial_imu_load::publishToTopics(){
    imuMsgsPublisher_ = nh_.advertise<sensor_msgs::Imu>(imu_msgs_topic_name_,1);
}
void serial_imu_load::sendMsg(){
    imuMsgsPublisher_.publish(serial_imu_load::getImuState());
}

short serial_imu_load::solver_byte2(int startIndex,int byteNum,uint8_t buffer[512]){
    char data[byteNum];
    for(int i = 0; i<byteNum; i++){
        data[i] = buffer[startIndex+i];
    }
    short dataProcessed_;
    memcpy(&dataProcessed_,data,byteNum);
    return dataProcessed_;
}

int serial_imu_load::solver_byte4(int startIndex,int byteNum,uint8_t buffer[512]){
    char data[byteNum];
    for(int i = 0; i<byteNum; i++){
        data[i] = buffer[startIndex+i];
    }
    int dataProcessed_;
    memcpy(&dataProcessed_,data,byteNum);
    return dataProcessed_;
}

}
