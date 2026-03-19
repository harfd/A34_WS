#include <ros/ros.h>
#include "estimationState_handle.hpp"

namespace ns_estimationState {

// Constructor
EstimationStateHandle::EstimationStateHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int EstimationStateHandle::getNodeRate() const { return node_rate_; }

// Methods
void EstimationStateHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("imu_state_topic_name",
                                      imu_state_topic_name_,
                                      "/sensor/imu")) {
    ROS_WARN_STREAM("Did not load imu_state_topic_name. Standard value is: " << imu_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("tf_info_sub_topic_name",
                                      tf_info_sub_topic_name_,
                                      "/tf/info")) {
    ROS_WARN_STREAM("Did not load tf_info_sub_topic_name. Standard value is: " << tf_info_sub_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("imu_coord_sub_topic_name",
                                      imu_coord_sub_topic_name_,
                                      "/imu/coord")) {
    ROS_WARN_STREAM("Did not load imu_coord_sub_topic_name. Standard value is: " << imu_coord_sub_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("localization_way_sub_topic_name",
                                      localization_way_sub_topic_name_,
                                      "/localization/way")) {
    ROS_WARN_STREAM("Did not load localization_way_sub_topic_name. Standard value is: " << localization_way_sub_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("cone_map_sub_topic_name",
                                      cone_map_sub_topic_name_,
                                      "/cone_map")) {
    ROS_WARN_STREAM("Did not load cone_map_sub_topic_name. Standard value is: " << cone_map_sub_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("mission_topic_name",
                                      mission_topic_name_,
                                      "/mission")) {
    ROS_WARN_STREAM("Did not load mission_topic_name. Standard value is: " << mission_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("car_state_topic_name",
                                      car_state_topic_name_,
                                      "/estimation/slam/state")) {
    ROS_WARN_STREAM("Did not load car_state_topic_name. Standard value is: " << car_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("local_map_topic_name",
                                      local_map_topic_name_,
                                      "/local_map")) {
    ROS_WARN_STREAM("Did not load local_map_topic_name. Standard value is: " << local_map_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
  if (!nodeHandle_.param("coordSystemNum", coordSystemNum_, 4)) {
    ROS_WARN_STREAM("Did not load coordSystemNum. Standard value is: " << coordSystemNum_);
  }
  if (!nodeHandle_.param<std::string>("slamCoordFile", slamCoordFile_,"/home/cris/data_record/coord_slam.txt")) {
    ROS_WARN_STREAM("Did not load slamCoordFile. Standard value is: " << slamCoordFile_);
  }
  if (!nodeHandle_.param<std::string>("imuCoordFile", imuCoordFile_,"/home/cris/data_record/coord_imu.txt")) {
    ROS_WARN_STREAM("Did not load imuCoordFile. Standard value is: " << imuCoordFile_);
  }
}

void EstimationStateHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  imuStateSubscriber_ = nodeHandle_.subscribe(imu_state_topic_name_, 1, &EstimationStateHandle::imuStateCallback, this);
  tfInfoSubscriber_ = nodeHandle_.subscribe(tf_info_sub_topic_name_, 1, &EstimationStateHandle::tfInfoCallback, this);
  imuCoordSubscriber_ = nodeHandle_.subscribe(imu_coord_sub_topic_name_, 1, &EstimationStateHandle::imuCoordCallback, this);
  localizationWaySubscriber_ = nodeHandle_.subscribe(localization_way_sub_topic_name_, 1, &EstimationStateHandle::localizationWayCallback, this);
  coneMapSubscriber_ = nodeHandle_.subscribe(cone_map_sub_topic_name_, 1, &EstimationStateHandle::coneMapCallback, this);
  missionSubscriber_ = nodeHandle_.subscribe(mission_topic_name_, 1, &EstimationStateHandle::missionCallback, this);
}

void EstimationStateHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  carStatePublisher_  = nodeHandle_.advertise<fsd_common_msgs::CarState>(car_state_topic_name_, 1);
  localMapPublisher_ = nodeHandle_.advertise<fsd_common_msgs::Map>(local_map_topic_name_, 1);
}

void EstimationStateHandle::run() {
  sendMsg();
}

void EstimationStateHandle::sendMsg() {
  if (mission_ == "acceleration" || mission_ == "skidpad") { 
    carStatePublisher_.publish(EstimationStateHandle::setCarState());
  }
  else if (mission_ == "trackdrive"){
    carStatePublisher_.publish(EstimationStateHandle::setTrackdriveCarState());
    localMapPublisher_.publish(EstimationStateHandle::setLocalMap());
  }
}

void EstimationStateHandle::imuStateCallback(const coordSystem::imu_state &msg) {
    imu_state_ = msg;
}
void EstimationStateHandle::tfInfoCallback(const sensor_msgs::PointCloud &msg) {
    tf_info_ = msg;
}
void EstimationStateHandle::imuCoordCallback(const geometry_msgs::Point &msg) {
    imu_coord_ = msg;
}
void EstimationStateHandle::localizationWayCallback(const std_msgs::String &msg) {
    localization_ = msg.data;
}
void EstimationStateHandle::coneMapCallback(const fsd_common_msgs::Map &msg) {
    cone_map_ = msg;
}
void EstimationStateHandle::missionCallback(const std_msgs::String &msg) {
    mission_ = msg.data;
}

fsd_common_msgs::CarState EstimationStateHandle::setCarState() {
  if (tf_info_.points.size() >= 3) {
    
    if (localization_ == "slam") {
      try {
        listener_.lookupTransform("fssim_map","base_link_vehicle",ros::Time(0),transform_);
        car_state_.car_state_dt.car_state_dt.x = imu_state_.N_vel;
        car_state_.car_state_dt.car_state_dt.y = imu_state_.E_vel;
        car_state_.car_state.x = transform_.getOrigin().x();
        car_state_.car_state.y = transform_.getOrigin().y();
        car_state_.car_state.theta = tf_info_.points[1].z - tf_info_.points[2].z;

        ROS_INFO("VehicleState v : %.2f(m/s)  coordinate x : %.2f(m) coordiate y : %.2f(m) yaw angle %.6f(rad)",
              imu_state_.vel,car_state_.car_state.x,car_state_.car_state.y,car_state_.car_state.theta);

        if (!is_init_data_ && ros::Time::now().toSec()!=0) {
        coordFileSlam_.open(slamCoordFile_, std::ofstream::app);
        std::ofstream file_writer(slamCoordFile_, std::ios_base::out);
        time_init_ = ros::Time::now().toSec();
        is_init_data_ = true;
        }
        

        else if (is_init_data_) {
            coordFileSlam_ << transform_.getOrigin().x() << "    " << transform_.getOrigin().y() <<std::endl;
        }
      }
      catch (tf::TransformException ex) {
        ROS_WARN("The tf doesn't work successfully !");
      }

    }
    else if (localization_ == "imu") {
      try {
        geometry_msgs::PointStamped imu_coord_imu_map;
        imu_coord_imu_map.header.frame_id = "imu_map";
        imu_coord_imu_map.header.stamp = ros::Time();
        imu_coord_imu_map.point.x = imu_coord_.x;
        imu_coord_imu_map.point.y = imu_coord_.y;
        geometry_msgs::PointStamped imu_coord_fssim_map_;
        listener1_.transformPoint("fssim_map",imu_coord_imu_map,imu_coord_fssim_map_);
        double X_trans = imu_coord_fssim_map_.point.x;
        double Y_trans = imu_coord_fssim_map_.point.y;

        car_state_.car_state_dt.car_state_dt.x = imu_state_.N_vel;
        car_state_.car_state_dt.car_state_dt.y = imu_state_.E_vel;
        car_state_.car_state.x = X_trans;
        car_state_.car_state.y = Y_trans;
        car_state_.car_state.theta = tf_info_.points[1].z - tf_info_.points[2].z;

        ROS_INFO("VehicleState v : %.2f(m/s)  coordinate x : %.2f(m) coordiate y : %.2f(m) yaw angle %.6f(rad)",
              imu_state_.vel,car_state_.car_state.x,car_state_.car_state.y,car_state_.car_state.theta);

        if (!is_init_data_ && ros::Time::now().toSec()!=0) {
          coordFileImu_.open(imuCoordFile_, std::ofstream::app);
          std::ofstream file_writer1(imuCoordFile_, std::ios_base::out);
          time_init_ = ros::Time::now().toSec();
          is_init_data_ = true;
        }

        else if (is_init_data_) {
          coordFileImu_ << X_trans << "    " << Y_trans <<std::endl;
        }
      }
      catch (tf::TransformException ex) {
        ROS_WARN("The tf doesn't work successfully !");
      }
    }

  }
  else {
    ROS_WARN("Wait for the tf_broadcaster!");
  }
  return car_state_;
}

fsd_common_msgs::CarState EstimationStateHandle::setTrackdriveCarState() {
  if (tf_info_.points.size() >= 2 ) {
    car_state_.car_state_dt.car_state_dt.x = imu_state_.N_vel;
    car_state_.car_state_dt.car_state_dt.y = imu_state_.E_vel;
    car_state_.car_state.x = 0;
    car_state_.car_state.y = 0;
    car_state_.car_state.theta = tf_info_.points[1].z;
    ROS_INFO("VehicleState v : %.2f(m/s)  coordinate x : %.2f(m) coordiate y : %.2f(m) yaw angle %.6f(rad)",
          imu_state_.vel,car_state_.car_state.x,car_state_.car_state.y,car_state_.car_state.theta);
  }
  return car_state_;
}

fsd_common_msgs::Map EstimationStateHandle::setLocalMap() {
  if (tf_info_.points.size() != 0) {
    try {
      int red_size = cone_map_.cone_red.size();
      int blue_size = cone_map_.cone_blue.size();
      int unknown_size = cone_map_.cone_unknow.size();
      if ((red_size+blue_size+unknown_size) > 4) {
        local_map_.cone_red.clear();
        local_map_.cone_blue.clear();
        local_map_.cone_unknow.clear();
      }
      std::cout << "red size : " << red_size << std::endl;
      std::cout << "blue size: " << blue_size << std::endl;
      std::cout << "unknow size: " << unknown_size << std::endl;  
      
      for (int i = 0;i < red_size;i++) {
        geometry_msgs::PointStamped current_cone;
        geometry_msgs::PointStamped trans_cone;
        current_cone.header.frame_id = "laser_link";
        current_cone.point.x = cone_map_.cone_red[i].position.x;
        current_cone.point.y = cone_map_.cone_red[i].position.y;
        listener_.transformPoint("base_link_vehicle",current_cone,trans_cone);
        cone_map_.cone_red[i].position.x = trans_cone.point.x;
        cone_map_.cone_red[i].position.y = trans_cone.point.y;
        local_map_.cone_red.push_back(cone_map_.cone_red[i]);
      }
      for (int i = 0;i < blue_size;i++) {
        geometry_msgs::PointStamped current_cone;
        geometry_msgs::PointStamped trans_cone;
        current_cone.header.frame_id = "laser_link";
        current_cone.point.x = cone_map_.cone_blue[i].position.x;
        current_cone.point.y = cone_map_.cone_blue[i].position.y;
        listener_.transformPoint("base_link_vehicle",current_cone,trans_cone);
        cone_map_.cone_blue[i].position.x = trans_cone.point.x;
        cone_map_.cone_blue[i].position.y = trans_cone.point.y;
        local_map_.cone_blue.push_back(cone_map_.cone_blue[i]);
      }
      for (int i = 0; i < unknown_size; i++) {
        geometry_msgs::PointStamped current_cone;
        geometry_msgs::PointStamped trans_cone;
        current_cone.header.frame_id = "laser_link";
        current_cone.point.x = cone_map_.cone_unknow[i].position.x;
        current_cone.point.y = cone_map_.cone_unknow[i].position.y;
        listener_.transformPoint("base_link_vehicle",current_cone,trans_cone);
        cone_map_.cone_unknow[i].position.x = trans_cone.point.x;
        cone_map_.cone_unknow[i].position.y = trans_cone.point.y;
        local_map_.cone_unknow.push_back(cone_map_.cone_unknow[i]);
      }
    }
    catch (tf::TransformException ex) {
      ROS_WARN("The tf doesn't work successfully !");
    } 
  }
  return local_map_;
}

}