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

#include "lidar_cluster.hpp"
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <utility>


namespace ns_lidar_cluster {
// Constructor
LidarCluster::LidarCluster(ros::NodeHandle &nh) : nh_(nh) { loadParameters(); };

// load Param
void LidarCluster::loadParameters() {
  getRawLidar_ = false;
  is_ok_flag_ = false;

  if (!nh_.param("Cluster_Tolerance", Cluster_Tolerance_, 0.5)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Cluster_Tolerance_);
  }

  if (!nh_.param("MinClusterSize", MinClusterSize_, 2)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << MinClusterSize_);
  }

  if (!nh_.param("MaxClusterSize", MaxClusterSize_, 1000)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << MaxClusterSize_);
  }

  if (!nh_.param("MinRange", MinRange_, 0.5)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << MinRange_);
  }

  if (!nh_.param("MaxRange", MaxRange_, 25.0)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << MaxRange_);
  }

  if (!nh_.param("Min_x", Min_x_, -0.5)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Min_x_);
  }

  if (!nh_.param("Max_z", Max_z_, 0.7)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Max_z_);
  }

  if (!nh_.param("Max_bound_x", Max_bound_x_, 0.5)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Max_bound_x_);
  }

  if (!nh_.param("Max_bound_y", Max_bound_y_, 0.5)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Max_bound_y_);
  }

  if (!nh_.param("Min_bound_z", Min_bound_z_, -0.08)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Min_bound_z_);
  }
  
  if (!nh_.param("Max_bound_z", Max_bound_z_, 0.7)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << Max_bound_z_);
  }

  if (!nh_.param("max_centroid_height", max_centroid_height_, 0.7)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << max_centroid_height_);
  }


}

// Getters
sensor_msgs::PointCloud LidarCluster::getConesCentroid() { return cones_centroid_; }

sensor_msgs::PointCloud2 LidarCluster::getFilterCones() { return filter_cones_; }

sensor_msgs::PointCloud2 LidarCluster::getFilterGround() { return filter_ground_; }

sensor_msgs::PointCloud2 LidarCluster::getFilterRaw() { return filter_raw_; }

sensor_msgs::PointCloud2 LidarCluster::getGreedyCloud() { return cloud_greedy_; }

sensor_msgs::PointCloud2 LidarCluster::getClusterCones() { return cones_cloud_; }

lidar_cluster::ClusterCones LidarCluster::getClusterMsgs() {return Cones_;}

visualization_msgs::MarkerArray  LidarCluster::getRvizVisual() {return cones_marker_;}



bool LidarCluster::is_ok() const { return is_ok_flag_; }

// Setters
void LidarCluster::setRawLidar(const sensor_msgs::PointCloud2 &msg) {
  raw_pc2_ = msg;
  getRawLidar_ = true;
}

void LidarCluster::runAlgorithm() {
  if (raw_pc2_.fields.empty() || !getRawLidar_) {
    return;
  }
  getRawLidar_ = false;

  pcl::fromROSMsg(raw_pc2_, raw_pc_);

  // 
  std::vector<int>  rm_indices; //保存去除的点的索引
  raw_pc_.is_dense = false;
  pcl::removeNaNFromPointCloud(raw_pc_, raw_pc_, rm_indices);

  //Debug //check if there is NaN piont
  // ofstream out("/home/shawn/file.txt");  
  // out << raw_pc_.is_dense << endl;

  //   for (auto cone_idx : raw_pc_.points)
  // {
  //   out << "raw" << cone_idx << endl ;
  // }
  

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones(
      new pcl::PointCloud<pcl::PointXYZI>);


  // segment the groud point cloud and get the raw_flatten
  preprocessing(raw_pc_, cloud_ground, cloud_cones);
 
  // use cluster and multi filter to get the cones position
  ClusterProcessing(cloud_cones, Cluster_Tolerance_); //default Cluster_Tolerance_ = 0.5

  cones_centroid_.header.frame_id = "laser_link";
  cones_centroid_.header.stamp = raw_pc2_.header.stamp;
  is_ok_flag_ = true;
}

void LidarCluster::preprocessing(
    pcl::PointCloud<pcl::PointXYZI> &raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones) {

  pcl::PointCloud<pcl::PointXYZI> filtered;

//  std::chrono::steady_clock::time_point   t1 = std::chrono::steady_clock::now();

  for (auto &iter : raw.points) {
    if (std::hypot(iter.x, iter.y) < MinRange_ || std::hypot(iter.x, iter.y) > MaxRange_
        || iter.z > 0.7 || iter.x < Min_x_)
        //(std::hypot(iter.x, iter.y) > 7 && iter.z < 0.03)||
      continue;
    filtered.points.push_back(iter);
  }
  
  // std::chrono::steady_clock::time_point   t2 = std::chrono::steady_clock::now();
  // double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
  // std::cout<<"filter_time: "<<time_round<<std::endl;
  
  filtered.width = filtered.points.size();
  filtered.height = 1;
  filtered.is_dense = false ;
  //debug
  // pcl::io::savePCDFileASCII("filtered.pcd" , filtered);
  
  // ofstream out("/home/shawn/file.txt");  
  // out << "raw_count" << raw.size() << endl;
  // out << "filtered_count" << filtered.size() << endl ;
  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // set Distance Threshold
  seg.setDistanceThreshold(0.07);//0.07
  seg.setInputCloud(filtered.makeShared());
  seg.segment(*inliers, *coefficients);

  //out << "inliers_count" << inliers->indices.size() << endl ;
  /* Debug 
    for (auto iter : coefficients->values) {
      std::cout << iter << " ";
    }
    std::cout << "\n-------------\n";
  */ 
  
  
  // extract ground
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(filtered.makeShared());
  extract.setIndices(inliers);
  extract.filter(*cloud_ground);

  // extract cone
  extract.setNegative(true);
  extract.filter(*cloud_cones);

  pcl::toROSMsg(*cloud_ground, filter_ground_);
  pcl::toROSMsg(*cloud_cones, filter_cones_);
  filter_ground_.header.frame_id = "laser_link";
  filter_ground_.header.stamp = ros::Time::now();
  filter_cones_.header.frame_id = "laser_link";
  filter_cones_.header.stamp = ros::Time::now(); 
  // pcl::io::savePCDFileASCII("cloud_ground.pcd" , *cloud_ground);
  // pcl::io::savePCDFileASCII("cloud_cones.pcd" , *cloud_cones);
  /* Debug
  for (auto cone_idx : cloud_cones->points)
  {
    std::cout << cone_idx << endl ;
  }
  */
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
	// pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cones_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::copyPointCloud(*cloud_cones ,*cloud_cones_tmp);
	// tree2->setInputCloud(cloud_cones_tmp);   ///用cloud构建tree对象
	// n.setInputCloud(cloud_cones_tmp);
	// n.setSearchMethod(tree2);
	// n.setKSearch(20);
	// n.compute(*normals);       ////估计法线存储到其中
	// //* normals should not contain the point normals + surface curvatures

	// // Concatenate the XYZ and normal fields*
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	// pcl::concatenateFields(*cloud_cones_tmp, *normals, *cloud_with_normals);    //连接字段
	// //* cloud_with_normals = cloud + normals

	// //定义搜索树对象
	// pcl::search::KdTree<pcl::PointNormal>::Ptr tree3(new pcl::search::KdTree<pcl::PointNormal>);
	// tree3->setInputCloud(cloud_with_normals);   //点云构建搜索树

	// // Initialize objects
	// pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
	// pcl::PolygonMesh triangles;                //存储最终三角化的网络模型

	// // Set the maximum distance between connected points (maximum edge length)
	// gp3.setSearchRadius(0.025);  //设置连接点之间的最大距离，（即是三角形最大边长）0.025

	// // 设置各参数值
	// gp3.setMu(3);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
	// gp3.setMaximumNearestNeighbors(100);    //设置样本点可搜索的邻域个数
	// gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45
	// gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10
	// gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120
	// gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致

	// // Get result
	// gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	// gp3.setSearchMethod(tree3);   //设置搜索方式
	// gp3.reconstruct(triangles);  //重建提取三角化
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_triangles(new pcl::PointCloud<pcl::PointXYZ>);
	// fromPCLPointCloud2(triangles.cloud, *cloud_triangles);
	// // *cloud_cones_tmp = *cloud_triangles;
	// pcl::toROSMsg(*cloud_triangles, cloud_greedy_);
	// cloud_greedy_.header.frame_id = "laser_link";
	// pub_cloud_greedy_.publish(cloud_greedy);

}

void LidarCluster::ClusterProcessing(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold) {

  cones_centroid_.points.clear();

  std::chrono::steady_clock::time_point   t1 = std::chrono::steady_clock::now();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(threshold);
  ec.setMinClusterSize(MinClusterSize_);
  ec.setMaxClusterSize(MaxClusterSize_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
   
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cone(new pcl::PointCloud<pcl::PointXYZI>);
  lidar_cluster::ClusterCone temp_cone;
  marker_count_ = 0;
  Cones_.ClusterCones.clear();

  std::chrono::steady_clock::time_point   t2 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
  std::cout<<"cluster_time: "<<time_round<<std::endl;


  t1 = std::chrono::steady_clock::now();
  
  for (const auto &iter : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cone(new pcl::PointCloud<pcl::PointXYZI>);
    
    for (auto it : iter.indices) {
      cone->points.push_back(cloud->points[it]);
      cluster_cone->points.push_back(cloud->points[it]);
    }
    cone->width = cone->points.size();
    cone->height = 1;
    cone->is_dense = true;
    
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::compute3DCentroid(*cone, centroid);
    pcl::getMinMax3D(*cone, min, max);

    float bound_x = std::fabs(max[0] - min[0]);
    float bound_y = std::fabs(max[1] - min[1]);
    float bound_z = std::fabs(max[2] - min[2]);
    
    // filter based on the shape of cones
    //(bound_x < 0.5 && bound_y < 0.5 && bound_z < 0.7 && bound_z > 0 && centroid[2] < 0.8)
    if (bound_x < Max_bound_x_ && bound_y < Max_bound_y_ && bound_z < Max_bound_z_ && bound_z > Min_bound_z_ &&centroid[2] < max_centroid_height_) {
      geometry_msgs::Point32 tmp;
      tmp.x = centroid[0];
      tmp.y = centroid[1];
      tmp.z = centroid[2];
      // std::cout << "x:" << tmp.x << "y:" << tmp.y << "z:" << tmp.z << endl;
      // std::cout << "bound_x:" << bound_x << "bound_y:" << bound_y << "bound_z:" << bound_z << endl;
      temp_cone.xmax = max[0];
      temp_cone.xmin = min[0];
      temp_cone.ymax = max[1];
      temp_cone.ymin = min[1];
      temp_cone.zmax = max[2];
      temp_cone.zmin = min[2];

      Cones_.ClusterCones.push_back(temp_cone);
      cones_centroid_.points.push_back(tmp);

      LidarCluster::VisualCones(tmp,min,max);
      // std::cout << "marker_count: " << marker_count_ << std::endl;
    }

  }

  t2 = std::chrono::steady_clock::now();
  time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
  std::cout<<"load_time: "<<time_round<<std::endl;

    cluster_cone->width = cluster_cone->points.size();
    cluster_cone->height = 1;
    cluster_cone->is_dense = true;
    pcl::toROSMsg(*cluster_cone, cones_cloud_);
    cones_cloud_.header.frame_id = "laser_link";
    cones_cloud_.header.stamp = ros::Time::now();
    // pcl::io::savePCDFileASCII("cluster_cone.pcd" , *cluster_cone);

    Cones_.header.stamp = ros::Time::now();
}

void LidarCluster::VisualCones(geometry_msgs::Point32 tmp,Eigen::Vector4f min, Eigen::Vector4f max){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "laser_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = marker_count_;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = tmp.x;
  marker.pose.position.y = tmp.y;
  marker.pose.position.z = tmp.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = std::fabs(max[0] - min[0]);
  marker.scale.y = std::fabs(max[1] - min[1]);
  marker.scale.z = std::fabs(max[2] - min[2]);
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(0.5);
  cones_marker_.markers.push_back(marker);
  marker_count_ += 1;
}

} // namespace ns_lidar_cluster
