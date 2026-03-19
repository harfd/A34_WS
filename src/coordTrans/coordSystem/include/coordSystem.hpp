#ifndef COORDSYSTEM_HPP
#define COORDSYSTEM_HPP

#include "std_msgs/String.h"
#include <math.h>
#include <proj.h>
#include "coordSystem/imu_state.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "solver/line_fitting.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Point.h"
#include "opencv2/imgproc.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "type.hpp"
#include "visual.hpp"
#include "tf/tf.h"
namespace ns_coordSystem {

class CoordSystem {

 public:
  // Constructor
  CoordSystem(ros::NodeHandle& nh);

	// Getters
  sensor_msgs::PointCloud getTfInfo();
  std_msgs::Int8 getErrorSignal();
  geometry_msgs::Point getImuCoord();
  std_msgs::String getLocalizationWay();
  std_msgs::String getMission();
  visualization_msgs::Marker getTriangles();
  visualization_msgs::Marker getPath();
	// Setters
  void setImuState(coordSystem::imu_state msg);
  void setCenterPoints(sensor_msgs::PointCloud msg);
  void setSlamState(geometry_msgs::Pose2D msg);

  //run
  void showImuState();
  void setInitialState();
  void coordTrans();
  void chooseMidPoints();
  void line_fitting_init();
  void tf_skeleton();
  void runAlgorithm();
  void clearimuInfo();
  void checkImuError();

private:

	ros::NodeHandle& nh_;
	
  coordSystem::imu_state imu_state_;
  sensor_msgs::PointCloud center_points;
  sensor_msgs::PointCloud center_points_deal;
  geometry_msgs::Pose2D slam_state_;
  sensor_msgs::PointCloud tf_info_;
  geometry_msgs::Point imu_coord_;
  bool check_imformation_imu_;
  bool check_imformation_coordtrans_;
  bool is_init  = false;
  bool is_init_line = false;
  double x_projection;
	double y_projection;
  float yaw_init_;
  double x_projection_init_;
  double y_projection_init_;
  std::vector<float> init_data_vec;
  int index = 0;
  float initialX = 0;
  float initialY = 0;
  solver solver_;
  solver solver1_;
  float line_k = 0;
  float line_b = 0;
  float line_correlation_coefficient = 0;
  geometry_msgs::Point32 laser_link2base_link;
  geometry_msgs::Point32 slam_map2laser_link;
  geometry_msgs::Point32 slam_map2map;
  geometry_msgs::Point32 imu_map2imu_link;
  float laser2base_length_;
  std_msgs::Int8 imu_error_;
  std::string localization_way_;
  std::string mission_;
  std_msgs::String way_;
  std_msgs::String Mission_;
  int fitting_size_;
  double X_trans;
  double Y_trans;
  visualization_msgs::Marker triangles_;
  visualization_msgs::Marker path_;
  geometry_msgs::PoseArray line_list_;
};
}

#endif //COORDSYSTEM_HPP
