# Lidar Cluster

Shawn Pan

本功能包基于北理BITFSD开源代码进行部分修改

This ROS package is an open source version of lidar_cluster from BITFSD. We removed some components to keep it simple but it still has great performance.  

## 1 Algorithm

The core idea of this algorithm is euclidean clustering. First, we filter out the point clouds on the ground, and filter out the point clouds that meet the feature of the cones, then cluster them.

### **version: BITSFD**

- 获得原始点云之后去除NaN无效点云（原始点云属性`is_dense`需要为`false`）
- 去除地面点云的时候，输入应该是过滤后的`filtered`，而不是`raw`（源代码错误）

### **version: 1.1**

增加输出：

1. **cloud_cones.pcd**	去除地面点云之后的点云（默认为锥桶点云）
2. **cloud_ground.pcd**  RANSAC检测的地面点云
3. **cluster_cone.pcd**    聚类之后的锥桶点云
4. **filtered.pcd**              对原始点云进行距离，角度判断之后去除的点云

### version: 1.2

参考`darknet`的BoundingBox和BoundingBoxes，自定义消息格式

存放每一帧雷达数据的聚类信息

```python
#ClusterCones
Header header
ClusterCone[] ClusterCones

#ClusterCone
int16 id
float64 xmax #聚类的锥桶边界值 x y z
float64 ymax
float64 zmax
float64 xmin
float64 ymin
float64 zmin
sensor_msgs/PointCloud2 pointcloud
```
### version: 1.3
| 变量名          | 内容                               | 话题                       |
| --------------- | ---------------------------------- | -------------------------- |
| cones_centroid_ | 聚类并根据形状过滤后的锥桶质心坐标 | /perception/cones_centroid |
| filter_cones_   | 去除地面后的点云                   | /perception/filter_cones   |
| filter_ground_  | 提取的地面点云                     |                            |
| cones_cloud_    | 聚类后的锥桶点云                   | /perception/cluster_cones  |
| Cones_          | 包含点云簇包围框信息的聚类点云簇   | /perception/clustermsgs    |

**可视化处理：**

使用 `visualization_msgs::Marker` 可视化点云包围框

## 2 Important Dependencies

* ROS

* PCL

  * 安装教程参考：[Ubuntu18.04下安装点云库PCL1.9.0](https://blog.csdn.net/czychen1997/article/details/107306674)，[Ubuntu18.04安装PCL（详细教程）](https://blog.csdn.net/weixin_45629790/article/details/112547011?utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromMachineLearnPai2%7Edefault-1.control&dist_request_id=&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromMachineLearnPai2%7Edefault-1.control)

  * 首先安装[metslib-0.5.3](https://www.coin-or.org/download/source/metslib/metslib-0.5.3.tgz) ，[VTK-8.2.0.tar.gz](https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz) 这两个库。

  * 如果遇到下面这个问题，那就是Qt版本太高了，重新下载Qt5.0-5.5都可！

    ```bash
     Could not find a package configuration file provided by "Qt5WebKitWidgets"
      with any of the following names:
    Qt5WebKitWidgetsConfig.cmake
    qt5webkitwidgets-config.cmake
    ```



## 3 ROS topic

* **subscriber:**
  - /velodyne_points (sensor_msgs/PointCloud2)
* **publisher:**	
  - /perception/cones_centroid (sensor_msgs/PointCloud)



## 4 Key parameters

All parameters are set in the `./config/lidar_cluster.yaml` 

## 5 Prerequisites

You need a LiDAR sensor to generate point clouds.

## 6 Step

* Move package to your workspace.
* Go to your workspace,  `catkin build`
* `source devel/setup.bash`
* `roslaunch lidar_cluster lidar_cluster.launch`

