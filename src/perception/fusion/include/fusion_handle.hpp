#ifndef FUSION_HANDLE_HPP
#define FUSION_HANDLE_HPP

#include "fusion.hpp"

namespace ns_fusion {

class FusionHandle {

 public:
  // Constructor
  FusionHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
  void getCalibration_Param();

  void TimesyncCallback(const sensor_msgs::ImageConstPtr &image_msg ,
                        const fusion::ClusterConesConstPtr  &lidar_msg,
                        const fusion::BoundingBoxes::ConstPtr &bbox_msg);

  void rawimageCallback(const sensor_msgs::ImageConstPtr &rawimage_msg);
  void lidarCallback(const sensor_msgs::PointCloud2 &cluster_cone_msg);
  void pathCallback(const visualization_msgs::MarkerArray &path_msg);

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber rawimageSubscriber_;
  ros::Subscriber lidarSubscriber_;
  ros::Subscriber pathSubscriber_;

  message_filters::Subscriber<sensor_msgs::Image> imageSubscriber_;
  // message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSubscriber_;
  message_filters::Subscriber<fusion::BoundingBoxes> bboxSubscriber_;
  message_filters::Subscriber<fusion::ClusterCones> lidar_bbox_sub_;

  // ros::Subscriber  imageSubscriber_;
  // ros::Subscriber  lidarSubscriber_;

  typedef message_filters::sync_policies::ApproximateTime
            <sensor_msgs::Image ,fusion::ClusterCones, fusion::BoundingBoxes> syncPolicy_;
  //message_filters::Synchronizer<syncPolicy_> sync_;
  message_filters::Synchronizer<syncPolicy_> sync_{syncPolicy_(10)};
  ros::Publisher color_cloud_Pub_;
  ros::Publisher color_marker_Pub_;
  ros::Publisher map_pub_;
  image_transport::Publisher fusion_img_pub_;

  
  std::string image_topic_name_;
  std::string lidar_topic_name_;
  std::string lidar_bbox_topic_name_;
  std::string color_cloud_topic_name_;
  std::string color_marker_topic_name_;
  std::string calibration_file_;
  std::string bbox_topic_name_;
  

  int node_rate_;

  Fusion fusion_;

};
}

#endif //FUSION_HANDLE_HPP
