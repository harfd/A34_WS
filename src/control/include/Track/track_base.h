#pragma once
#include "Basic/types.h"
#include "Basic/param.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include "fsd_tools/cubic_spline.h"
#include <iostream>
#include <fstream>

namespace ns_control {
   class Track
   {
   public:

   void setTrackEndPoint(const geometry_msgs::Point &endPoint);
   void setPlanningPath(geometry_msgs::PoseArray &planningTraj);
   void setTrackVehicleState(const VehicleState &state);

   virtual bool getTraj() = 0;
   virtual bool getTrajPath(Trajectory &TrajPath) = 0;
   virtual bool calTraj(Trajectory &relativePath) = 0;

   protected:
   geometry_msgs::Point endPoint_;
   geometry_msgs::PoseArray planningTraj_;
   VehicleState state_;
   Trajectory trajectory_;
   double lateralDis_;
   std::vector<double> lateralDisVec_;
   std::ofstream lateralDisVecFile_;
   bool is_init_ = false;
   double time_init_;
   };
}