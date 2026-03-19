/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef LIDAR_CLUSTER_HPP
#define LIDAR_CLUSTER_HPP

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include <chrono>
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

#include <ClusterCone.h>
#include <ClusterCones.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <time.h>


namespace ns_lidar_cluster {

class LidarCluster {

 public:
  // Constructor
  LidarCluster(ros::NodeHandle &nh);

  // Getters
  // sensor_msgs::PointCloud getLidarCluster();
  sensor_msgs::PointCloud getConesCentroid();
  sensor_msgs::PointCloud2 getFilterCones();
  sensor_msgs::PointCloud2 getFilterGround();
  sensor_msgs::PointCloud2 getFilterRaw();
  sensor_msgs::PointCloud2 getClusterCones();
  lidar_cluster::ClusterCones getClusterMsgs();
  visualization_msgs::MarkerArray  getRvizVisual();
  sensor_msgs::PointCloud2 getGreedyCloud();

  bool is_ok() const;

  // Setters
  void setRawLidar(const sensor_msgs::PointCloud2 &msg);

  void runAlgorithm();

 private:
  ros::NodeHandle &nh_;

  void loadParameters();
  double Cluster_Tolerance_;
  int MinClusterSize_;
  int MaxClusterSize_;
  double MinRange_;
  double MaxRange_;
  double Min_x_;
  double Max_z_;
  double Max_bound_x_;
  double Max_bound_y_;
  double Min_bound_z_;
  double Max_bound_z_;
  double max_centroid_height_;

  bool getRawLidar_, is_ok_flag_;

  sensor_msgs::PointCloud cones_centroid_;

  sensor_msgs::PointCloud2 raw_pc2_;
  
  sensor_msgs::PointCloud2 filter_ground_, filter_cones_ ,filter_raw_ ,cones_cloud_;
  
  sensor_msgs::PointCloud2 cloud_greedy_;
            
  pcl::PointCloud<pcl::PointXYZI> raw_pc_;
  pcl::PointCloud<pcl::PointXYZI> raw_pc_rm_;

  lidar_cluster::ClusterCones Cones_;

  visualization_msgs::MarkerArray cones_marker_;
  int marker_count_;

  void preprocessing(pcl::PointCloud<pcl::PointXYZI> &raw,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones);
                     
  void ClusterProcessing(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold);

  void VisualCones(geometry_msgs::Point32 tmp,Eigen::Vector4f min, Eigen::Vector4f max);
};
} // namespace ns_lidar_cluster

#endif // LIDAR_CLUSTER_HPP
