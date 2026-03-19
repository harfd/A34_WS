#ifndef FUSION_HPP
#define FUSION_HPP

// sys
#include <math.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>

// ros
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "chrono"

// opencv
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types.hpp"

// ros
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// custom_msgs
#include <fusion/BoundingBox.h>
#include <fusion/BoundingBoxes.h>
#include <fusion/ObjectCount.h>
#include <fusion/ClusterCone.h>
#include <fusion/ClusterCones.h>
#include <fsd_common_msgs/Cone.h>
#include <fsd_common_msgs/Map.h>

// 3rd library
#include <Hungarian.h>


using namespace std;

namespace ns_fusion {

class Fusion {

 public:
  // Constructor
  explicit Fusion(ros::NodeHandle& nh);


  void loadParameters();
  bool visual_flag_;
  double cone_bbox_area_max_;
  double cone_bbox_area_min_;
  double h_w_ratio_min_;

	// Setters
  void setFusion_msg(const sensor_msgs::ImageConstPtr& image_msg ,
                     const fusion::ClusterConesConstPtr &lidar_msg,
                     const fusion::BoundingBoxes::ConstPtr &bbox_msg);
  
  void setCluster_Cone_msg(const sensor_msgs::PointCloud2 &cluster_cone_msg);

  void setBbox();
  
  void setPath_msg(const visualization_msgs::MarkerArray &path_msg);

  void getcalibration(const std::string& calibration_config_yaml);

  //publish return
  sensor_msgs::PointCloud2 getColorCloud();
  visualization_msgs::MarkerArray getColorMarker();
  fsd_common_msgs::Map getFusionMap(); 
  sensor_msgs::ImagePtr getFusionImage();

  double GetIOU(cv::Rect_<float> img_bbox, cv::Rect_<float> cone_bbox);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_association(int img_idx, int cone_idx);
  void VisualCones(Eigen::Vector3f min, Eigen::Vector3f max,float r,float g,float b,int img_idx ,int cone_idx);
  void color_marker(int img_idx, int cone_idx);
  void close_sort(fsd_common_msgs::Map &map);
  void project_visual();
  int runAlgorithm();

private:

	ros::NodeHandle& nh_;
  
  cv_bridge::CvImagePtr cam_image_;
  pcl::PointCloud<pcl::PointXYZRGB> cluster_cones_;

  cv::Mat cameraextrinsicmat_; //外参
  cv::Mat intrisicMat_;          //内参
  cv::Mat distCoeffs_;          //畸变系数
  cv::Size imagesize_;
  cv::Mat rVec_;
  cv::Mat tVec_;

  fusion::ClusterCones lidar_bbox_msgs_;
  fusion::ClusterCones raw_lidar_bbox_msgs_;
  fusion::BoundingBoxes img_bbox_msgs_;
  
  vector<cv::Rect_<float>> image_bboxes_;
  vector<cv::Rect_<float>> cones_bboxes_;
  vector<vector<double>> iouMatrix;
  vector<int> assignment;
  set<int> unmatched_img_bbox_;
  set<int> unmatched_cones_bbox_;
  set<int> all_idx_;
  set<int> matched_idx_;
  vector<cv::Point> matched_img_cones_;
  double match_iou_;
  pcl::PointCloud<pcl::PointXYZRGB> color_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB> color_point_;
  sensor_msgs::PointCloud2 color_cloud_msgs_;
  
  visualization_msgs::MarkerArray cones_marker_;
  int marker_count_ = 0;

  fsd_common_msgs::Map fusion_map_;
  sensor_msgs::ImagePtr fusion_image_;

  std::vector<cv::Point3f> path_points_;
  std::vector<cv::Point2f> projected_path_points_;
};
}

#endif //FUSION_HPP
