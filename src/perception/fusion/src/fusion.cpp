/*
version:    4.5
author:     Shawn Pan 
Time:       2021.10.24 22:05

debug: 对lidar_bbox_msgs容器erase，会造成索引不对齐的问题，新建了lidar_bbox_msgs和raw_lidar_bbox_msgs_区分
增加投影框可视化图像话题/fusion_img,可视化cone_idx,img_idx
rviz可视化marker：配对的cone_idx以及img_idx，未配对的显示-1，cone_idx
增加对投影回图像的cone_bbox阈值
更改融合点云显示为pointcloud<XYZRGB>
*/

#include <ros/ros.h>
#include <fusion.hpp>
#include <sstream>
#include <fstream>

namespace ns_fusion {
// Constructor
Fusion::Fusion(ros::NodeHandle &nh) : nh_(nh) {
  loadParameters();
};

void Fusion::loadParameters(){
  if (!nh_.param("match_iou", match_iou_, 0.3)) {
    ROS_WARN_STREAM("Did not load Cluster_Tolerance. Standard value is: " << match_iou_);
  }
  if (!nh_.param("project_visual", visual_flag_, false)) {
    ROS_WARN_STREAM("visual_flag is: " << visual_flag_);
  }

  if (!nh_.param("cone_bbox/area_max", cone_bbox_area_max_, 0.25)) {
    ROS_WARN_STREAM("cone_bbox_area_max is: " << cone_bbox_area_max_);
  }

 if (!nh_.param("cone_bbox/area_min", cone_bbox_area_min_, 0.1)) {
    ROS_WARN_STREAM("cone_bbox_area_min is: " << cone_bbox_area_min_);
  }

  if (!nh_.param("cone_bbox/h_w_ratio_min", h_w_ratio_min_, 0.5)) {
    ROS_WARN_STREAM("cone_bbox_area_min is: " << h_w_ratio_min_);
  }

}


// 同步点云bbox_msgs和图像bbox_msgs
void Fusion::setFusion_msg(const sensor_msgs::ImageConstPtr& image_msg ,
                           const fusion::ClusterConesConstPtr  &lidar_msg,
                           const fusion::BoundingBoxes::ConstPtr &bbox_msg) {
  //set image
  try {
    cam_image_ = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  //set bbox
  img_bbox_msgs_ = *bbox_msg;
  raw_lidar_bbox_msgs_ = *lidar_msg;
}

// 装载cones_bboxes_和image_bboxes_
void Fusion::setBbox(){
  if (!raw_lidar_bbox_msgs_.ClusterCones.empty() && !img_bbox_msgs_.bounding_boxes.empty() ){

    cv::MatSize img_size = cam_image_->image.size;
    // img_height = img_size[0]
    // img_weidth = img_size[1]
    // img   -------> x
    //       |
    //       |
    //       |
    //       y

    std::vector<cv::Point3f> objectPoints;
    for (auto & cone_bbox : raw_lidar_bbox_msgs_.ClusterCones) {
      objectPoints.emplace_back( cv::Point3f(cone_bbox.xmin,cone_bbox.ymin,cone_bbox.zmin) );
      objectPoints.emplace_back( cv::Point3f(cone_bbox.xmax,cone_bbox.ymax,cone_bbox.zmax) );
    }

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(objectPoints, rVec_, tVec_, intrisicMat_, distCoeffs_, projectedPoints);

    //Rect(int left_top_x, int left_top_y, int width, int height);
    cones_bboxes_.clear();
    lidar_bbox_msgs_.header = raw_lidar_bbox_msgs_.header;
    lidar_bbox_msgs_.ClusterCones.clear();
    for (int i = 0; i < projectedPoints.size(); i+=2){
      /* for debug */
      // std::cout << i << " at "<< projectedPoints.size() << std::endl;
      // std::cout << projectedPoints[i].x << " " << projectedPoints[i].y << " " << projectedPoints[i+1].x << " " << projectedPoints[i+1].y << " " << std::endl;
      // std::cout << "img_size: " << img_size[0] << "x" << img_size[1] << std::endl;
      // std::cout << "lidar_bbox_msgs_ClusterCones_size: " << lidar_bbox_msgs_.ClusterCones.size()  << std::endl;
      
      cv::Rect_<float> temp_bbox(projectedPoints[i+1].x,
                       projectedPoints[i+1].y,
                       std::abs(projectedPoints[i+1].x - projectedPoints[i].x),
                       std::abs(projectedPoints[i+1].y - projectedPoints[i].y));
      cv::Rect_<float> img_size_box(0,0,img_size[1],img_size[0]);
      cv::Rect_<float> cone_bbox = img_size_box & temp_bbox;

      // if(projectedPoints[i].x > img_size[1] && projectedPoints[i+1].x > img_size[1]
      //   && projectedPoints[i].y > img_size[0] && projectedPoints[i+1].y > img_size[0] ){

      //   fsd_common_msgs::Cone fusion_cones;
      //   fusion_map_.cone_unknow.push_back(fusion_cones);

      //   pcl::PointXYZRGB tmp_color_point;
      //   tmp_color_point.x = fusion_cones.position.x;
      //   tmp_color_point.y = fusion_cones.position.y;
      //   tmp_color_point.z = fusion_cones.position.z;
      //   tmp_color_point.r = 0;
      //   tmp_color_point.g = 0;
      //   tmp_color_point.b = 0;
      //   color_point_.push_back(tmp_color_point);

      //   lidar_bbox_msgs_.ClusterCones.erase(lidar_bbox_msgs_.ClusterCones.begin() + (i/2) );
      // }
      // else{
        if(cone_bbox.area() > cone_bbox_area_min_ 
          && cone_bbox.area() < img_size_box.area() * cone_bbox_area_max_ 
          && cone_bbox.height/cone_bbox.width >= h_w_ratio_min_ )
        {
          cones_bboxes_.push_back(cone_bbox);
          lidar_bbox_msgs_.ClusterCones.push_back(raw_lidar_bbox_msgs_.ClusterCones[i/2]);

        }
        else{
          fsd_common_msgs::Cone fusion_cones;
          Eigen::Vector3f min(raw_lidar_bbox_msgs_.ClusterCones[i/2].xmin,
                              raw_lidar_bbox_msgs_.ClusterCones[i/2].ymin,
                              raw_lidar_bbox_msgs_.ClusterCones[i/2].zmin);
          Eigen::Vector3f max(raw_lidar_bbox_msgs_.ClusterCones[i/2].xmax,
                              raw_lidar_bbox_msgs_.ClusterCones[i/2].ymax,
                              raw_lidar_bbox_msgs_.ClusterCones[i/2].zmax);
          fusion_cones.position.x = (min[0] + max[0])/2;
          fusion_cones.position.y = (min[1] + max[1])/2;
          fusion_cones.position.z = (min[2] + max[2])/2;
          fusion_map_.cone_unknow.push_back(fusion_cones);
          
          pcl::PointXYZRGB tmp_color_point;
          tmp_color_point.x = fusion_cones.position.x;
          tmp_color_point.y = fusion_cones.position.y;
          tmp_color_point.z = fusion_cones.position.z;
          tmp_color_point.r = 0;
          tmp_color_point.g = 0;
          tmp_color_point.b = 0;
          color_point_.push_back(tmp_color_point);

          // raw_lidar_bbox_msgs_.ClusterCones.erase(raw_lidar_bbox_msgs_.ClusterCones.begin() + (i/2) );
        }
      // }
      
    }
  
    image_bboxes_.clear();
    for (auto &bbox : img_bbox_msgs_.bounding_boxes){
      cv::Rect_<float> temp_bbox(bbox.xmin,
                                 bbox.ymin,
                                 std::abs(bbox.xmax - bbox.xmin),
                                 std::abs(bbox.ymax - bbox.ymin));
      image_bboxes_.push_back(temp_bbox);
    }

  }
  else{
    std::cout << "no bboxes!" << std::endl;
  }
}


// 获取聚类点云信息
void Fusion::setCluster_Cone_msg(const sensor_msgs::PointCloud2 &cluster_cone_msg){
  pcl::PointCloud<pcl::PointXYZI> cones_xyzi ;
  pcl::fromROSMsg(cluster_cone_msg,cones_xyzi);
  pcl::copyPointCloud(cones_xyzi ,color_cloud_);
  std::cout << "color_cloud_: " << color_cloud_.size() << std::endl; 
  printf("lidar_msgs_ok");
}

void Fusion::setPath_msg(const visualization_msgs::MarkerArray &path_msg){
  path_points_.clear();
  for (auto &path : path_msg.markers){
    for (auto &path_point : path.points){
      std::cout << "path_point:  "<< path_point.x << "   "<< path_point.y << std::endl;
      path_points_.emplace_back(cv::Point3f(path_point.x,path_point.y,0));
      
    }
  }
  if (!path_points_.empty()){
    cv::projectPoints(path_points_, rVec_, tVec_, intrisicMat_, distCoeffs_, projected_path_points_);
  }
  
}

// 获取标定参数
void Fusion::getcalibration(const std::string& calibration_config_yaml){
    cv::FileStorage fs1(calibration_config_yaml,cv::FileStorage::READ);
    fs1["CameraExtrinsicMat"] >> cameraextrinsicmat_;
    fs1["CameraMat"] >> intrisicMat_;
    fs1["DistCoeff"] >> distCoeffs_;
    fs1["ImageSize"] >> imagesize_;


    cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    cv::Mat rMat = cameraextrinsicmat_(cv::Rect(0, 0, 3, 3)).t();

    Rodrigues(rMat,rVec);

    tVec.at<double>(0,0) = cameraextrinsicmat_.at<double>(1,3);
    tVec.at<double>(1,0) = cameraextrinsicmat_.at<double>(2,3);
    tVec.at<double>(2,0) = - cameraextrinsicmat_.at<double>(0,3);
    rVec_ = rVec;
    tVec_ = tVec;
    
}

//publish return
sensor_msgs::PointCloud2 Fusion::getColorCloud() {return color_cloud_msgs_;}

visualization_msgs::MarkerArray Fusion::getColorMarker() {return cones_marker_;}

fsd_common_msgs::Map Fusion::getFusionMap() {return fusion_map_;}

sensor_msgs::ImagePtr Fusion::getFusionImage() {return fusion_image_;}
//publish return end

// 计算IOU
double Fusion::GetIOU(cv::Rect_<float> img_bbox, cv::Rect_<float> cone_bbox){
  //diou = 1 - iou + d(A,B)^2 / c^2
  //in 交集面积
	float in = (cone_bbox & img_bbox).area();
  //un 并集面积
	float un = cone_bbox.area() + img_bbox.area() - in;
  double iou;

	if (un < DBL_EPSILON){
    iou = 0;
  }
  else{
    iou = (double)(in / un);
  }
    
  cv::Rect min_rect = cone_bbox | img_bbox;
  double c2 = pow(min_rect.size().height,2) + pow(min_rect.size().width,2);
  
  cv::Point2f img_bbox_center = cv::Point2f( (img_bbox.tl().x + img_bbox.br().x)/2 , (img_bbox.tl().y + img_bbox.br().y)/2 );
  cv::Point2f cone_bbox_center = cv::Point2f( (cone_bbox.tl().x + cone_bbox.br().x)/2 , (cone_bbox.tl().y + cone_bbox.br().y)/2 );
  double d2 = pow((img_bbox_center.x - cone_bbox_center.x),2) +  pow((img_bbox_center.y - cone_bbox_center.y),2) ;
  double diou;
  if (c2 < DBL_EPSILON){
    diou = iou;
  }
  else{
    diou = iou - d2/c2;
  }
  
  return diou;
}


void Fusion::VisualCones(Eigen::Vector3f min, Eigen::Vector3f max,float r,float g,float b,int img_idx ,int cone_idx){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "laser_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = marker_count_;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position.x = (min[0] + max[0])/2;
  // marker.pose.position.y = (min[1] + max[1])/2;
  // marker.pose.position.z = (min[2] + max[2])/2;
  marker.pose.position.x = max[0];
  marker.pose.position.y = max[1];
  marker.pose.position.z = max[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // marker.scale.x = std::fabs(max[0] - min[0]);
  // marker.scale.y = std::fabs(max[1] - min[1]);
  // marker.scale.z = std::fabs(max[2] - min[2]);
  marker.scale.z = 0.2;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 255;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(0.5);
  std::ostringstream str;
  if (img_idx < 0){
    str << "img_idx: " << img_idx << "\n" <<"cone_idx: " << cone_idx << "\n" << "no match img_idx";
  }else{
    str << "img_idx: " << img_idx << "\n" <<"cone_idx: " << cone_idx << "\n" << img_bbox_msgs_.bounding_boxes[img_idx].Class;
  }
  marker.text = str.str();
  cones_marker_.markers.push_back(marker);
  marker_count_ += 1;
}

void Fusion::color_marker(int img_idx, int cone_idx){
  fsd_common_msgs::Cone fusion_cones;
  pcl::PointXYZRGB tmp_color_point;
  Eigen::Vector3f min(lidar_bbox_msgs_.ClusterCones[cone_idx].xmin,
                      lidar_bbox_msgs_.ClusterCones[cone_idx].ymin,
                      lidar_bbox_msgs_.ClusterCones[cone_idx].zmin);
  Eigen::Vector3f max(lidar_bbox_msgs_.ClusterCones[cone_idx].xmax,
                      lidar_bbox_msgs_.ClusterCones[cone_idx].ymax,
                      lidar_bbox_msgs_.ClusterCones[cone_idx].zmax);
  if (img_bbox_msgs_.bounding_boxes[img_idx].Class =="red"){
    VisualCones(min,max,255,0,0,img_idx,cone_idx);
    fusion_cones.position.x = (min[0] + max[0])/2;
    fusion_cones.position.y = (min[1] + max[1])/2;
    fusion_cones.position.z = (min[2] + max[2])/2;
    fusion_cones.color.data = "r";
    fusion_map_.cone_red.push_back(fusion_cones);

    tmp_color_point.x = fusion_cones.position.x;
    tmp_color_point.y = fusion_cones.position.y;
    tmp_color_point.z = fusion_cones.position.z;
    tmp_color_point.r = 255;
    tmp_color_point.g = 0;
    tmp_color_point.b = 0;
    color_point_.push_back(tmp_color_point);

  }
  else if (img_bbox_msgs_.bounding_boxes[img_idx].Class =="blue"){
    VisualCones(min,max,0,0,255,img_idx,cone_idx);
    fusion_cones.position.x = (min[0] + max[0])/2;
    fusion_cones.position.y = (min[1] + max[1])/2;
    fusion_cones.position.z = (min[2] + max[2])/2;
    fusion_cones.color.data = "b";
    fusion_map_.cone_blue.push_back(fusion_cones);

    tmp_color_point.x = fusion_cones.position.x;
    tmp_color_point.y = fusion_cones.position.y;
    tmp_color_point.z = fusion_cones.position.z;
    tmp_color_point.r = 0;
    tmp_color_point.g = 0;
    tmp_color_point.b = 255;
    color_point_.push_back(tmp_color_point);
  }
  else if (img_bbox_msgs_.bounding_boxes[img_idx].Class =="yellow"){
    VisualCones(min,max,255,255,0,img_idx,cone_idx);
    fusion_cones.position.x = (min[0] + max[0])/2;
    fusion_cones.position.y = (min[1] + max[1])/2;
    fusion_cones.position.z = (min[2] + max[2])/2;
    fusion_cones.color.data = "y";
    fusion_map_.cone_yellow.push_back(fusion_cones);

    tmp_color_point.x = fusion_cones.position.x;
    tmp_color_point.y = fusion_cones.position.y;
    tmp_color_point.z = fusion_cones.position.z;
    tmp_color_point.r = 255;
    tmp_color_point.g = 255;
    tmp_color_point.b = 0;
    color_point_.push_back(tmp_color_point);

  }
  else if (img_bbox_msgs_.bounding_boxes[img_idx].Class =="big_yellow"){
    VisualCones(min,max,255,255,0,img_idx,cone_idx);
    fusion_cones.position.x = (min[0] + max[0])/2;
    fusion_cones.position.y = (min[1] + max[1])/2;
    fusion_cones.position.z = (min[2] + max[2])/2;
    fusion_cones.color.data = "by";
    fusion_map_.cone_big_yellow.push_back(fusion_cones);

    tmp_color_point.x = fusion_cones.position.x;
    tmp_color_point.y = fusion_cones.position.y;
    tmp_color_point.z = fusion_cones.position.z;
    tmp_color_point.r = 0;
    tmp_color_point.g = 255;
    tmp_color_point.b = 255;
    color_point_.push_back(tmp_color_point);
  }
}

void Fusion::close_sort(fsd_common_msgs::Map &map_){
  if(map_.cone_red.size() > 1){
      std::sort(map_.cone_red.begin(),map_.cone_red.end(),[&](fsd_common_msgs::Cone a,fsd_common_msgs::Cone b){
                                  double d_a = pow(a.position.x,2) + pow(a.position.y,2);
                                  double d_b = pow(b.position.x,2) + pow(b.position.y,2);
                                  return d_a < d_b;
                                });
  }

  if(map_.cone_blue.size() > 1){
  std::sort(map_.cone_blue.begin(),map_.cone_blue.end(),[&](fsd_common_msgs::Cone a,fsd_common_msgs::Cone b){
                                double d_a = pow(a.position.x,2) + pow(a.position.y,2);
                                double d_b = pow(b.position.x,2) + pow(b.position.y,2);
                                return d_a < d_b;
                                });
  }

  if(map_.cone_yellow.size() > 1){
  std::sort(map_.cone_yellow.begin(),map_.cone_yellow.end(),[&](fsd_common_msgs::Cone a,fsd_common_msgs::Cone b){
                                  double d_a = pow(a.position.x,2) + pow(a.position.y,2);
                                  double d_b = pow(b.position.x,2) + pow(b.position.y,2);
                                  return d_a < d_b;
                                  });
  }

  if(map_.cone_big_yellow.size() > 1){
  std::sort(map_.cone_big_yellow.begin(),map_.cone_big_yellow.end(),[&](fsd_common_msgs::Cone a,fsd_common_msgs::Cone b){
                                double d_a = pow(a.position.x,2) + pow(a.position.y,2);
                                  double d_b = pow(b.position.x,2) + pow(b.position.y,2);
                                  return d_a < d_b;
                                  }); 
  }

  if(map_.cone_unknow.size() > 1){
  std::sort(map_.cone_unknow.begin(),map_.cone_unknow.end(),[&](fsd_common_msgs::Cone a,fsd_common_msgs::Cone b){
                                double d_a = pow(a.position.x,2) + pow(a.position.y,2);
                                  double d_b = pow(b.position.x,2) + pow(b.position.y,2);
                                  return d_a < d_b;
                                  }); 
  }
}

void Fusion::project_visual(){
  int cones_bbox_count = 0;
  int img_bbox_cout = 0;

  // draw bbox
  for(const auto& bbox: image_bboxes_){
    // cv::rectangle(cam_image_->image,bbox,cv::Scalar(255,0,0),1,1,0);
    cv::putText(cam_image_->image,to_string(img_bbox_cout),bbox.tl(), cv::FONT_HERSHEY_DUPLEX,1, cv::Scalar(0,140,255),2,8,0); //cv::Scalar(BGRA) 
    img_bbox_cout++;
  }

  for(const auto& bbox: cones_bboxes_){
    cv::Point temp;
    temp.x = bbox.br().x;
    temp.y = bbox.br().y;
    cv::circle(cam_image_->image, temp, 4, cv::Scalar(255,255,0),-1);
    temp.x = bbox.tl().x;
    temp.y = bbox.tl().y;
    cv::circle(cam_image_->image, temp, 4, cv::Scalar(0,255,255),-1);
    cv::putText(cam_image_->image,to_string(cones_bbox_count),bbox.tl(), cv::FONT_HERSHEY_DUPLEX,1, cv::Scalar(0,255,0),2,8,0);
    cv::rectangle(cam_image_->image,bbox,cv::Scalar(0,255,0),1,1,0);
    cones_bbox_count++;
  }
  std::cout << "image_bboxes: "<< image_bboxes_.size() << std::endl;
  std::cout << "cones_bboxes: "<< cones_bboxes_.size() << std::endl;

  // projectPoints
  for(auto &point : color_cloud_){
    vector<cv::Point3f> objectPoints;
    objectPoints.emplace_back(cv::Point3f(point.x,point.y,point.z));
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(objectPoints, rVec_, tVec_, intrisicMat_, distCoeffs_, projectedPoints);
    cv::Point temp;
    temp.x = projectedPoints[0].x;
    temp.y = projectedPoints[0].y;
    cv::circle(cam_image_->image, temp, 3, cv::Scalar(255,0,0),-1);
  }

  for(auto point : projected_path_points_){
    cv::circle(cam_image_->image, cv::Point(point.x,point.y), 5, cv::Scalar(127,255,0),-1);
  }
  
  // make ros msgs
  color_point_.width = color_point_.size();
  color_point_.height = 1;
  color_point_.is_dense = true;
  pcl::toROSMsg(color_point_,color_cloud_msgs_);
  color_cloud_msgs_.header.frame_id = "laser_link";
  color_cloud_msgs_.header.stamp = ros::Time::now();

  fusion_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_image_->image).toImageMsg();
  fusion_image_->header.stamp = ros::Time::now();
}


int Fusion::runAlgorithm() {
  
  //clear msgs
  unmatched_img_bbox_.clear();
	unmatched_cones_bbox_.clear();
	all_idx_.clear();
	matched_idx_.clear();
  matched_img_cones_.clear();
  
  cones_marker_.markers.clear();
  marker_count_ = 0;
  fusion_map_.cone_blue.clear();
  fusion_map_.cone_yellow.clear();
  fusion_map_.cone_red.clear();
  fusion_map_.cone_big_yellow.clear();
  fusion_map_.cone_unknow.clear();
  color_point_.clear();
  

  setBbox();

  if (image_bboxes_.empty() && cones_bboxes_.empty() ){
    std::cout << "bboxes empty!" << std::endl;
    return 0;
  }
  
  int image_bboxes_size = image_bboxes_.size();
  int cones_bboxes_size = cones_bboxes_.size();
  iouMatrix.clear();
  iouMatrix.resize(image_bboxes_size,vector<double>(cones_bboxes_size,0));
  for (int i = 0; i < image_bboxes_size ; i++){
    for (int j = 0; j < cones_bboxes_size ; j++){
      iouMatrix[i][j] = 1 - GetIOU(image_bboxes_[i], cones_bboxes_[j]);
    }
  }
  HungarianAlgorithm HungAlgo;
	assignment.clear();
	HungAlgo.Solve(iouMatrix, assignment);
  /*output: img_index  assigenmnet[i]=coned_bbox_index
               0              3
               1              1
               2              0
               3              2
  */
  // for (int i = 0; i<assignment.size(); i++){
  //   std::cout << assignment[i] << std::endl;
  // }
  
		if (cones_bboxes_size > image_bboxes_size)  //there are unmatched cones_bboxes
		{
			for (unsigned int n = 0; n < cones_bboxes_size; n++)
				all_idx_.insert(n);

			// for (unsigned int i = 0; i < image_bboxes_size; ++i)
			// 	matched_idx_.insert(assignment[i]);

			set_difference(all_idx_.begin(), all_idx_.end(),
        assignment.begin(), assignment.end(),
				/*matched_idx_.begin(), matched_idx_.end(),*/
				insert_iterator<set<int>>(unmatched_cones_bbox_, unmatched_cones_bbox_.begin()));
		}
		else if (cones_bboxes_size < image_bboxes_size) //there are unmatched img_bboxes
		{
			for (unsigned int i = 0; i < image_bboxes_size; ++i)
				if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
					unmatched_img_bbox_.insert(i);
		}

		for (unsigned int i = 0; i < image_bboxes_size; ++i)
		{
			if (assignment[i] == -1) // pass over invalid values
				continue;
      // std::cout << "iou: " << 1 - iouMatrix[i][assignment[i]] << std::endl;

			if (1 - iouMatrix[i][assignment[i]] < match_iou_)
			{
        std::cout << "drap_iou: " << 1 - iouMatrix[i][assignment[i]] << std::endl;
				unmatched_img_bbox_.insert(i);
				unmatched_cones_bbox_.insert(assignment[i]);
			}
			else{
			  matched_img_cones_.push_back(cv::Point(i, assignment[i]));
        color_marker(i, assignment[i]);
        std::cout << "iou: " << 1 - iouMatrix[i][assignment[i]] << std::endl;
        std::cout << "image_idx: " << i << "  assignment to cone_idx: " << assignment[i] << std::endl;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_tmp = cloud_association(i, assignment[i]);
        // color_cloud_.insert(color_cloud_.end(),color_cloud_tmp.begin(),color_cloud_tmp.end());
      }
		}
    // for(auto unmatched_idx : unmatched_cones_bbox_ )
    // {
    //   fsd_common_msgs::Cone fusion_cones;
    //   Eigen::Vector3f min(lidar_bbox_msgs_.ClusterCones[unmatched_idx].xmin,
    //                       lidar_bbox_msgs_.ClusterCones[unmatched_idx].ymin,
    //                       lidar_bbox_msgs_.ClusterCones[unmatched_idx].zmin);
    //   Eigen::Vector3f max(lidar_bbox_msgs_.ClusterCones[unmatched_idx].xmax,
    //                       lidar_bbox_msgs_.ClusterCones[unmatched_idx].ymax,
    //                       lidar_bbox_msgs_.ClusterCones[unmatched_idx].zmax);
    //   VisualCones(min,max,255,255,255,-1,unmatched_idx); //白色
    //   fusion_cones.position.x = (min[0] + max[0])/2;
    //   fusion_cones.position.y = (min[1] + max[1])/2;
    //   fusion_cones.position.z = (min[2] + max[2])/2;
    //   fusion_map_.cone_unknow.push_back(fusion_cones);

    //     pcl::PointXYZRGB tmp_color_point;
    //     tmp_color_point.x = fusion_cones.position.x;
    //     tmp_color_point.y = fusion_cones.position.y;
    //     tmp_color_point.z = fusion_cones.position.z;
    //     tmp_color_point.r = 0;
    //     tmp_color_point.g = 0;
    //     tmp_color_point.b = 0;
    //     color_point_.push_back(tmp_color_point);
    // }

    close_sort(fusion_map_);
    fusion_map_.header.stamp = ros::Time::now();


  // std::cout << "color_cloud_size: " << color_cloud_.size() << std::endl;
  // int count = 0;
  
    if (visual_flag_){
      project_visual();
    }
      // //============//

    //   count +=1 ;
    //   for(int i = 0 ;i < image_bboxes_size; ++i){
    //     Eigen::Vector3f min(lidar_bbox_msgs_.ClusterCones[assignment[i]].xmin,
    //                         lidar_bbox_msgs_.ClusterCones[assignment[i]].ymin,
    //                         lidar_bbox_msgs_.ClusterCones[assignment[i]].zmin);
    //     Eigen::Vector3f max(lidar_bbox_msgs_.ClusterCones[assignment[i]].xmax,
    //                         lidar_bbox_msgs_.ClusterCones[assignment[i]].ymax,
    //                         lidar_bbox_msgs_.ClusterCones[assignment[i]].zmax);
    //     if (point.x > max[0] || point.x < min[0] || 
    //         point.y > max[1] || point.y < min[1] ||
    //         point.z > max[2] || point.z < min[2]){
    //         continue;
    //     }
    //     else{
    //       if(img_bbox_msgs_.bounding_boxes[i].Class =="red"){
    //         point.r = 255;
    //         point.g = 0;
    //         point.b = 0;
    //         // std::cout << "red" << std::endl;
    //         break;
    //       }
    //       if(img_bbox_msgs_.bounding_boxes[i].Class =="blue"){
    //         point.r = 0;
    //         point.g = 0;
    //         point.b = 255;
    //         // std::cout << "blue" << std::endl;
    //         break;
    //       }
    //       if(img_bbox_msgs_.bounding_boxes[i].Class =="yellow"){
    //         point.r = 255;
    //         point.g = 255;
    //         point.b = 0;
    //         // std::cout << "yelow" << std::endl;
    //         break;
    //       }
    //       if(img_bbox_msgs_.bounding_boxes[i].Class =="big_yellow"){
    //         point.r = 255;
    //         point.g = 255;
    //         point.b = 0;
    //         break;
    //       }
    //     }
    //   }
    // }

}

}//namespace end
