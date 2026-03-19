#ifndef BOUNDARYDETECTOR_HANDLE_HPP
#define BOUNDARYDETECTOR_HANDLE_HPP

#include "boundaryDetector.hpp"

namespace ns_boundaryDetector {

class BoundaryDetectorHandle {

 public:
  // Constructor
  BoundaryDetectorHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber localMapSubscriber;

  ros::Publisher boundaryDetectionsPublisher;
  ros::Publisher visualTrianglesPublisher;
  ros::Publisher visualBoundaryPublisher;
  ros::Publisher visualTreePublisher;
  ros::Publisher visualPathPublisher;
  ros::Publisher visualTrajPublisher;
  ros::Publisher planningTrajPublisher;
  void localMapCallback(const fsd_common_msgs::Map &msg);

  std::string local_map_topic_name_;
  std::string boundary_detections_topic_name_;
  std::string visual_triangles_topic_name_;
  std::string visual_boundary_topic_name_;
  std::string visual_tree_topic_name_;
  std::string visual_path_topic_name_;
  std::string visual_traj_topic_name_;
  std::string planning_traj_topic_name_;

  int node_rate_;

  BoundaryDetector boundaryDetector_;

};
}

#endif //BOUNDARYDETECTOR_HANDLE_HPP
